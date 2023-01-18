#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace util {

template <typename Clock> class PeriodicScheduler;

namespace impl {
template <typename Clock> void notifyScheduler(PeriodicScheduler<Clock>& scheduler);
}

/**
 * @brief Uses a single thread to periodically invoke callbacks at a given frequency.
 *
 * The underlying thread is initialized upon construction,
 * so do not create PeriodicSchedulers that will not be used.
 *
 * @tparam Clock The clock to use for scheduling.
 */
template <typename Clock = std::chrono::steady_clock> class PeriodicScheduler {
public:
	/**
	 * @brief Create a new PeriodicScheduler.
	 */
	PeriodicScheduler()
		: newEventAdded(false), quitting(false),
		  thread(std::bind(&PeriodicScheduler::threadFn, this)) {}

	/**
	 * @brief Join the thread and destruct.
	 */
	~PeriodicScheduler() {
		{
			std::unique_lock lock(scheduleMutex);
			quitting = true;
			newEventAdded = true;
		}
		scheduleCV.notify_one();
		thread.join();
	}

	/**
	 * @brief Clears all currently scheduled recurring events.
	 */
	void clear() {
		std::unique_lock lock(scheduleMutex);
		schedule = decltype(schedule)();
	}

	/**
	 * @brief Schedule a new event to be executed periodically.
	 *
	 * The first execution will be scheduled to happen one period from now.
	 *
	 * @param period The period in between executions of the event.
	 * @param fn The function to call when the event is executed. This should not block.
	 */
	void scheduleEvent(std::chrono::milliseconds period, const std::function<void()>& fn) {
		{
			std::unique_lock lock(scheduleMutex);
			auto now = Clock::now();
			schedule.push({now + period, period, fn});
			newEventAdded = true;
		}
		scheduleCV.notify_one();
	}

private:
	friend void util::impl::notifyScheduler<>(PeriodicScheduler&);
	void threadFn() {
		std::unique_lock lock(scheduleMutex);
		while (!quitting) {
			if (schedule.empty()) {
				scheduleCV.wait(lock, [&] { return newEventAdded; });
				newEventAdded = false;
			} else {
				auto now = Clock::now();
				schedule_t ts = schedule.top();
				if (ts.nextSendTime <= now) {
					schedule.pop();
					ts.fn();
					ts.nextSendTime += ts.period;
					schedule.push(ts);
				} else {
					scheduleCV.wait_until(lock, ts.nextSendTime,
										  [&] { return newEventAdded; });
					newEventAdded = false;
				}
			}
		}
	}

	struct schedule_t {
		std::chrono::time_point<Clock> nextSendTime;
		std::chrono::milliseconds period;
		std::function<void()> fn;
		friend bool operator>(const PeriodicScheduler::schedule_t& t1,
							  const PeriodicScheduler::schedule_t& t2) {
			return t1.nextSendTime > t2.nextSendTime;
		}
	};

	bool newEventAdded;
	bool quitting;
	std::thread thread;
	// we need a min priority queue
	std::priority_queue<schedule_t, std::vector<schedule_t>, std::greater<schedule_t>>
		schedule;
	std::mutex scheduleMutex;
	std::condition_variable scheduleCV;
};

namespace impl {

/**
 * @brief Notify the scheduler. This should not be used by client code.
 *
 * This is useful to trigger the scheduler to continue if the clock has been changed
 * and the scheduler hasn't woken up. Overusing this method can degrade performance.
 *
 * @tparam Clock The clock of the scheduler.
 * @param scheduler The scheduler to notify.
 *
 * @warning Client code should NOT use this.
 */
template <typename Clock> void notifyScheduler(PeriodicScheduler<Clock>& scheduler) {
	scheduler.scheduleCV.notify_all();
}

} // namespace impl

} // namespace util
