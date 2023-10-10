#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace util {

/**
 * @brief Implementation of a countdown latch for threading synchronization.
 *
 * This has roughly the same interface as std::latch, but that requires C++20,
 * which we do not support.
 *
 * @see https://en.cppreference.com/w/cpp/thread/latch
 */
class latch {
public:
	/**
	 * @brief Create a new latch.
	 *
	 * @param count This is the number of countdowns before the latch is unlocked.
	 */
	latch(std::size_t count);

	latch(const latch&) = delete;

	latch& operator=(const latch&) = delete;

	/**
	 * @brief Wait until the latch is unlocked.
	 */
	void wait() const;

	/**
	 * @brief Wait until the latch is unlocked, with a timeout.
	 *
	 * @param dur The timeout duration.
	 * @return bool True if the latch was unlocked, false if timed out.
	 */
	template <typename Rep, typename Period>
	bool wait_for(const std::chrono::duration<Rep, Period>& dur) const {
		std::unique_lock lock(mutex);
		return cv.wait_for(lock, dur, [&]() { return count == 0; });
	}

	/**
	 * @brief Wait until the latch is unlocked until a specific timepoint.
	 *
	 * @param tp The timepoint to wait until.
	 * @return bool True iff the latch was unlocked before the given time.
	 */
	template <typename Clock, typename Duration>
	bool wait_until(const std::chrono::time_point<Clock, Duration>& tp) const {
		std::unique_lock lock(mutex);
		return cv.wait_until(lock, tp, [&]() { return count == 0; });
	}

	/**
	 * @brief Counts down the internal counter.
	 *
	 * When the counter reaches 0 the latch is unlocked.
	 *
	 * @param n The amount to decrement by.
	 */
	void count_down(std::size_t n = 1);

private:
	std::size_t count;
	mutable std::mutex mutex;
	mutable std::condition_variable cv;
};

} // namespace util
