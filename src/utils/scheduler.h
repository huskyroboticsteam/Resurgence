#pragma once

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>

namespace util {

/**
 * @brief Uses a single thread to periodically invoke callbacks at a given frequency.
 *
 * The underlying thread is initialized upon construction,
 * so do not create PeriodicSchedulers that will not be used.
 */
class PeriodicScheduler {
public:
	/**
	 * @brief Create a new PeriodicScheduler.
	 */
	PeriodicScheduler();

	/**
	 * @brief Join the thread and destruct.
	 */
	~PeriodicScheduler();

	/**
	 * @brief Clears all currently scheduled recurring events.
	 */
	void clear();

	/**
	 * @brief Schedule a new event to be executed periodically.
	 *
	 * The first execution will be scheduled to happen one period from now.
	 *
	 * @param period The period in between executions of the event.
	 * @param fn The function to call when the event is executed. This should not block.
	 */
	void scheduleEvent(std::chrono::milliseconds period, const std::function<void()>& fn);

private:
	void threadFn();

	struct schedule_t {
		std::chrono::time_point<std::chrono::steady_clock> nextSendTime;
		std::chrono::milliseconds period;
		std::function<void()> fn;
	};
	friend bool operator>(const schedule_t&, const schedule_t&);

	bool newEventAdded;
	bool quitting;
	std::thread thread;
	// we need a min priority queue
	std::priority_queue<schedule_t, std::vector<schedule_t>, std::greater<schedule_t>>
		schedule;
	std::mutex scheduleMutex;
	std::condition_variable scheduleCV;
};

} // namespace util
