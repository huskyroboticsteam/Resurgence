#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <loguru.hpp>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>

namespace util {

namespace impl {

class Notifiable {
public:
	virtual void notify() = 0;
};

template <typename T>
void notifyScheduler(T& scheduler);
} // namespace impl

/**
 * @brief Uses a single thread to periodically invoke callbacks at a given frequency.
 *
 * The underlying thread is initialized upon construction,
 * so do not create PeriodicSchedulers that will not be used.
 *
 * @tparam Clock The clock to use for scheduling.
 */
template <typename Clock = std::chrono::steady_clock>
class PeriodicScheduler : private impl::Notifiable {
public:
	/**
	 * @brief The type of event ids.
	 */
	using eventid_t = uint64_t;

	/**
	 * @brief Create a new PeriodicScheduler.
	 *
	 * @param name The name of this scheduler, for logging purposes.
	 */
	explicit PeriodicScheduler(const std::optional<std::string>& name = std::nullopt)
		: name(name), newEventAdded(false), quitting(false), nextID(0),
		  thread(std::bind(&PeriodicScheduler::threadFn, this)) {}

	PeriodicScheduler(const PeriodicScheduler&) = delete;

	/**
	 * @brief Join the thread and destruct.
	 */
	~PeriodicScheduler() {
		{
			std::unique_lock lock(scheduleMutex);
			quitting = true;
			// wake up the thread with the cv so it can quit
			newEventAdded = true;
		}
		scheduleCV.notify_one();
		thread.join();
	}

	PeriodicScheduler& operator=(const PeriodicScheduler&) = delete;

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

	void notify() override {
		scheduleCV.notify_all();
	}

	/**
	 * @brief Method executed by the scheduler thread.
	 */
	void threadFn() {
		if (name.has_value()) {
			loguru::set_thread_name(name->c_str());
		}
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

	/**
	 * @brief Utility type for holding information about a scheduled event.
	 */
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

	std::optional<std::string> name;
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

/**
 * @brief Implements a thread-safe watchdog.
 *
 * A watchdog is a timer that is periodically reset (fed) by the client code. If the client
 * fails to feed the watchdog for some duration, then the watchdog is "starved", and the
 * callback is invoked. This is useful for implementing things such as heartbeats.
 *
 * @tparam Clock The clock to use for timing.
 *
 * @see https://en.wikipedia.org/wiki/Watchdog_timer
 */
template <typename Clock = std::chrono::steady_clock>
class Watchdog : private impl::Notifiable {
public:
	/**
	 * @brief Construct a new Watchdog.
	 *
	 * @param name The name of this Watchdog, for logging purposes.
	 * @param duration The timeout duration. If not fed for at least this long, then the
	 * callback is invoked.
	 * @param callback The callback to invoke when the watchdog starves.
	 * @param keepCallingOnDeath If true, keep invoking @p callback every @p duration
	 * milliseconds until fed again. Otherwise, only call @p callback when starved, and do not
	 * call again until being reset and subsequently starved again.
	 */
	Watchdog(const std::string& name, std::chrono::milliseconds duration,
			 const std::function<void()>& callback, bool keepCallingOnDeath = false)
		: name(name), duration(duration), callback(callback),
		  keepCallingOnDeath(keepCallingOnDeath), fed(false), quitting(false),
		  thread(std::bind(&Watchdog::threadFn, this)) {}

	/**
	 * @brief Construct a new Watchdog.
	 *
	 * @param duration The timeout duration. If not fed for at least this long, then the
	 * callback is invoked.
	 * @param callback The callback to invoke when the watchdog starves.
	 * @param keepCallingOnDeath If true, keep invoking @p callback every @p duration
	 * milliseconds until fed again. Otherwise, only call @p callback when starved, and do not
	 * call again until being reset and subsequently starved again.
	 */
	Watchdog(std::chrono::milliseconds duration, const std::function<void()>& callback,
			 bool keepCallingOnDeath = false)
		: name(std::nullopt), duration(duration), callback(callback),
		  keepCallingOnDeath(keepCallingOnDeath), fed(false), quitting(false),
		  thread(std::bind(&Watchdog::threadFn, this)) {}

	Watchdog(const Watchdog&) = delete;

	~Watchdog() {
		{
			std::lock_guard lock(mutex);
			quitting = true;
		}
		cv.notify_all();
		if (thread.joinable()) {
			thread.join();
		}
	}

	Watchdog& operator=(const Watchdog&) = delete;

	/**
	 * @brief Feed the watchdog.
	 *
	 * Call at least once per period, or the watchdog starves.
	 */
	void feed() {
		{
			std::lock_guard lock(mutex);
			fed = true;
		}
		cv.notify_one();
	}

private:
	friend void util::impl::notifyScheduler<>(Watchdog&);

	std::optional<std::string> name;
	std::chrono::milliseconds duration;
	std::function<void()> callback;
	const bool keepCallingOnDeath;
	bool fed;
	bool quitting;
	std::mutex mutex;
	std::condition_variable cv;
	std::thread thread;

	void notify() override {
		cv.notify_all();
	}

	void threadFn() {
		if (name.has_value()) {
			loguru::set_thread_name(name->c_str());
		}
		std::unique_lock lock(mutex);
		while (!quitting) {
			std::chrono::time_point<Clock> wakeTime = Clock::now() + duration;
			if (cv.wait_until(lock, wakeTime, [&]() { return fed || quitting; })) {
				if (quitting) {
					break;
				}
				fed = false;
			} else {
				callback();
				if (!keepCallingOnDeath) {
					cv.wait(lock, [&]() { return fed || quitting; });
				}
			}
		}
	}
};

/**
 * @brief An abstract class that can be overridden to run long-running tasks and encapsulate
 * task-related data.
 *
 * Client code should use this class by deriving from it and overriding AsyncTask::task.
 *
 * For simpler tasks that just require a function to be run periodically, consider
 * PeriodicTask.
 *
 * @tparam Clock The clock to use for timing.
 */
template <typename Clock = std::chrono::steady_clock>
class AsyncTask : private virtual impl::Notifiable {
public:
	/**
	 * @brief Construct a new task.
	 *
	 * @param name The name of this task, for logging purposes.
	 */
	AsyncTask(const std::optional<std::string>& name = std::nullopt)
		: name(name), running(false), quitting(false) {}

	AsyncTask(const AsyncTask&) = delete;

	virtual ~AsyncTask() {
		stop();
	}

	AsyncTask& operator=(const AsyncTask&) = delete;

	/**
	 * @brief Start the task.
	 *
	 * If the task is already running, do nothing.
	 */
	virtual void start() {
		std::lock_guard lock(mutex);
		if (!running) {
			if (thread.joinable()) {
				thread.join();
			}
			running = true;
			quitting = false;
			thread = std::thread(&AsyncTask::run, this);
		}
	}

	/**
	 * @brief Stop the task and wait for it to finish.
	 *
	 * If the task is not running, do nothing.
	 */
	virtual void stop() {
		bool isRunning = false;
		{
			std::lock_guard lock(mutex);
			if (isRunningInternal()) {
				quitting = true;
				isRunning = true;
			}
		}
		if (isRunning) {
			cv.notify_one();
			if (thread.joinable()) {
				thread.join();
			}
		}
	}

	/**
	 * @brief Check if the task is running.
	 *
	 * @return If the task is currently running.
	 */
	bool isRunning() {
		std::lock_guard lock(mutex);
		return running;
	}

protected:
	/**
	 * @brief The long-running task, overridden by client code.
	 *
	 * @param lock The lock on the private internal state of the AsyncTask. Client code should
	 * generally not use this except for the wait_until_xxx methods.
	 */
	virtual void task(std::unique_lock<std::mutex>& lock) = 0;

	/**
	 * @brief Version of AsyncTask::isRunning() that does no synchronization.
	 *
	 * This is useful if called from AsyncTask::task(), to prevent deadlocks.
	 *
	 * @return If the task is currently running.
	 */
	bool isRunningInternal() {
		return running;
	}

	/**
	 * @brief Wait until the specified time point, or until the task has been stopped.
	 *
	 * @param lock The lock passed to AsyncTask::task.
	 * @param tp The time point to wait until.
	 * @return true iff the task was stopped while waiting.
	 */
	bool wait_until(std::unique_lock<std::mutex>& lock,
					const std::chrono::time_point<Clock>& tp) {
		return cv.wait_until(lock, tp, [&]() { return quitting; });
	}

	/**
	 * @brief Wait for a given duration, or until the task has been stopped.
	 *
	 * @param lock The lock passed to AsyncTask::task.
	 * @param tp The duration of time to wait for.
	 * @return true iff the task was stopped while waiting.
	 */
	template <typename Rep, typename Period>
	bool wait_for(std::unique_lock<std::mutex>& lock,
				  const std::chrono::duration<Rep, Period>& dur) {
		return cv.wait_for(lock, dur, [&]() { return quitting; });
	}

	/**
	 * @brief Wait until the task has been stopped.
	 *
	 * @param lock The lock passed to AsyncTask::task.
	 */
	void wait_until_done(std::unique_lock<std::mutex>& lock) {
		return cv.wait(lock, [&]() { return quitting; });
	}

	/**
	 * @brief Not for use by client code.
	 */
	void notify() override {
		cv.notify_all();
	}

private:
	std::optional<std::string> name;
	bool running;
	bool quitting;
	std::thread thread;
	std::mutex mutex;
	std::condition_variable cv;

	friend void util::impl::notifyScheduler<>(AsyncTask&);

	void run() {
		if (name.has_value()) {
			loguru::set_thread_name(name->c_str());
		}
		std::unique_lock<std::mutex> lock(mutex);
		// dummy variable to use RAII to assign clear flags when exiting
		auto p = std::shared_ptr<int>(new int(0), [&](int* i) {
			running = false;
			quitting = false;
			delete i;
		});
		task(lock);
	}
};

/**
 * @brief Implements a task that executes a function periodically.
 *
 * Note that all PeriodicTask instances are run on the same thread, so the task function should
 * not block.
 *
 * @tparam Clock The clock to use for timing.
 */
template <typename Clock = std::chrono::steady_clock>
class PeriodicTask : public AsyncTask<Clock>, private virtual impl::Notifiable {
public:
	/**
	 * @brief Construct a new periodic task.
	 *
	 * @param period The period at which to run the task.
	 * @param f The function to execute every period.
	 */
	PeriodicTask(const std::chrono::milliseconds& period, const std::function<void()>& f)
		: AsyncTask<Clock>(), period(period), f(f) {}

protected:
	void task(std::unique_lock<std::mutex>& lock) override {
		auto event = scheduler.scheduleEvent(period, f);
		AsyncTask<Clock>::wait_until_done(lock);
		scheduler.removeEvent(event);
	}

	void notify() override {
		impl::notifyScheduler(scheduler);
		AsyncTask<Clock>::notify();
	}

private:
	std::chrono::milliseconds period;
	std::function<void()> f;

	inline static PeriodicScheduler<Clock> scheduler =
		PeriodicScheduler<Clock>("PeriodicTask_Scheduler");

	friend void util::impl::notifyScheduler<>(PeriodicTask&);
};

namespace impl {

/**
 * @brief Notify the scheduler. This should not be used by client code.
 *
 * This is useful to trigger the scheduler to continue if the clock has been changed
 * and the scheduler hasn't woken up. Overusing this method can degrade performance.
 *
 * @tparam T The type of the scheduler.
 * @param scheduler The scheduler to notify.
 *
 * @warning Client code should NOT use this.
 */
template <typename T>
void notifyScheduler(T& scheduler) {
	Notifiable& n = scheduler;
	n.notify();
}

} // namespace impl

} // namespace util
