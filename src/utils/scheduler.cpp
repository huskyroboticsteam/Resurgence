#include "scheduler.h"

namespace util {

bool operator>(const PeriodicScheduler::schedule_t& t1,
			   const PeriodicScheduler::schedule_t& t2) {
	return t1.nextSendTime > t2.nextSendTime;
}

PeriodicScheduler::PeriodicScheduler()
	: newEventAdded(false), quitting(false),
	  thread(std::bind(&PeriodicScheduler::threadFn, this)) {}

PeriodicScheduler::~PeriodicScheduler() {
	{
		std::unique_lock lock(scheduleMutex);
		quitting = true;
		newEventAdded = true;
	}
	scheduleCV.notify_one();
	thread.join();
}

void PeriodicScheduler::clear() {
	std::unique_lock lock(scheduleMutex);
	schedule = decltype(schedule)();
}

void PeriodicScheduler::scheduleEvent(std::chrono::milliseconds period,
									  const std::function<void()>& fn) {
	{
		std::unique_lock lock(scheduleMutex);
		auto now = std::chrono::steady_clock::now();
		schedule.push({now + period, period, fn});
		newEventAdded = true;
	}
	scheduleCV.notify_one();
}

void PeriodicScheduler::threadFn() {
	std::unique_lock lock(scheduleMutex);
	while (!quitting) {
		if (schedule.empty()) {
			scheduleCV.wait(lock, [&] { return newEventAdded; });
			newEventAdded = false;
		} else {
			auto now = std::chrono::steady_clock::now();
			schedule_t ts = schedule.top();
			if (ts.nextSendTime <= now) {
				schedule.pop();
				ts.fn();
				ts.nextSendTime += ts.period;
				schedule.push(ts);
			} else {
				scheduleCV.wait_until(lock, ts.nextSendTime, [&] { return newEventAdded; });
				newEventAdded = false;
			}
		}
	}
}

} // namespace util
