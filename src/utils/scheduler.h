#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_set>

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
	 * @brief The type of event ids.
	 */
	using eventid_t = uint64_t;

	/**
	 * @brief Create a new PeriodicScheduler.
	 */
	PeriodicScheduler()
		: newEventAdded(false), quitting(false), nextID(0),
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
	 *
	 * This invalidates all previously returned event ids.
	 */
	void clear() {
		std::unique_lock lock(scheduleMutex);
		schedule = decltype(schedule)();
		toRemove.clear();
		nextID = 0;
	}

	/**
	 * @brief Remove an event from the schedule.
	 *
	 * It is undefined behavior to remove events that are not scheduled,
	 * e.g. have already been removed.
	 *
	 * @param id The id of the event.
	 */
	void removeEvent(eventid_t id) {
		std::unique_lock lock(scheduleMutex);
		toRemove.insert(id);
	}

	/**
	 * @brief Schedule a new event to be executed periodically.
	 *
	 * The first execution will be scheduled to happen one period from now.
	 * It is undefined behavior to schedule more than 2^64 events.
	 *
	 * @param period The period in between executions of the event.
	 * @param fn The function to call when the event is executed. This should not block.
	 * @return The id of the scheduled event, which can be used with removeEvent().
	 */
	eventid_t scheduleEvent(std::chrono::milliseconds period,
							const std::function<void()>& fn) {
		eventid_t id;
		{
			std::unique_lock lock(scheduleMutex);
			id = nextID++;
			auto now = Clock::now();
			schedule.push({id, now + period, period, fn});
			newEventAdded = true;
		}
		scheduleCV.notify_one();
		return id;
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
				schedule_t event = schedule.top();
				if (toRemove.find(event.id) != toRemove.end()) {
					// if we should remove this id then remove the event and don't reschedule
					schedule.pop();
					toRemove.erase(event.id);
				} else if (event.nextSendTime <= now) {
					// execute the event and reschedule it
					schedule.pop();
					event.fn();
					event.nextSendTime += event.period;
					schedule.push(event);
				} else {
					// wait until the event should be executed
					scheduleCV.wait_until(lock, event.nextSendTime,
										  [&] { return newEventAdded; });
					newEventAdded = false;
				}
			}
		}
	}

	struct schedule_t {
		eventid_t id;
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
	eventid_t nextID = 0;
	std::thread thread;
	// we need a min priority queue
	std::priority_queue<schedule_t, std::vector<schedule_t>, std::greater<schedule_t>>
		schedule;
	std::mutex scheduleMutex;
	std::condition_variable scheduleCV;
	std::unordered_set<eventid_t> toRemove;
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
