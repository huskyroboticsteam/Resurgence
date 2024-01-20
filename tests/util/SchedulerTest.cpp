#include "../../src/utils/scheduler.h"

#include "../../src/utils/threading.h"

#include <chrono>
#include <memory>

#include <catch2/catch.hpp>

using namespace util;
using namespace std::chrono_literals;

namespace {

class fake_clock {
public:
	using duration = std::chrono::milliseconds;
	using rep = duration::rep;
	using period = duration::period;
	using time_point = std::chrono::time_point<fake_clock>;
	static inline constexpr bool is_steady = false;

	static void advance(duration d) noexcept {
		now_ += d;
	}
	static void reset_to_epoch() noexcept {
		now_ = decltype(now_)();
	}
	static time_point now() noexcept {
		return now_;
	}

private:
	fake_clock() = delete;
	~fake_clock() = delete;
	fake_clock(fake_clock const&) = delete;

	static inline time_point now_;
};

template <typename T>
void advanceFakeClock(std::chrono::milliseconds dur, std::chrono::milliseconds inc,
					  T& scheduler) {
	for (int i = 0; i < dur / inc; i++) {
		fake_clock::advance(inc);
		util::impl::notifyScheduler(scheduler);
	}
}
} // namespace

TEST_CASE("Test PeriodicScheduler", "[util][scheduler]") {
	PeriodicScheduler<fake_clock> sched;
	auto i = std::make_shared<latch>(2);
	auto j = std::make_shared<latch>(3);
	sched.scheduleEvent(100ms, [&]() { i->count_down(); });
	sched.scheduleEvent(75ms, [&]() { j->count_down(); });
	advanceFakeClock(250ms, 5ms, sched);
	// check that i was counted down twice and j thrice
	REQUIRE(i->wait_for(100ms));
	REQUIRE(j->wait_for(100ms));

	sched.clear();

	i = std::make_shared<latch>(1);
	j = std::make_shared<latch>(1);
	advanceFakeClock(250ms, 5ms, sched);
	// check that neither i nor j were counted down
	REQUIRE_FALSE(i->wait_for(100ms));
	REQUIRE_FALSE(j->wait_for(100ms));

	sched.scheduleEvent(100ms, [&]() { i->count_down(); });
	advanceFakeClock(150ms, 5ms, sched);
	// check that i was counted down but not j
	REQUIRE(i->wait_for(100ms));
	REQUIRE_FALSE(j->wait_for(100ms));
}

TEST_CASE("Test PeriodicScheduler Remove", "[util][scheduler]") {
	PeriodicScheduler<fake_clock> sched;
	auto i = std::make_shared<latch>(1);
	auto id = sched.scheduleEvent(100ms, [&]() { i->count_down(); });
	advanceFakeClock(100ms, 5ms, sched);

	REQUIRE(i->wait_for(100ms));

	sched.removeEvent(id);
	i = std::make_shared<latch>(1);
	advanceFakeClock(100ms, 5ms, sched);
	REQUIRE_FALSE(i->wait_for(100ms));
}

TEST_CASE("Test Watchdog", "[util][scheduler]") {
	// NOTE: after feeding the thread must wait for the watchdog thread to wake up and register
	// it. Doing feed() -> advanceTime() -> checkStarved() doesn't work. It should be feed() ->
	// checkFed() -> advanceTime() -> checkStarved().
	SECTION("Test regular") {
		auto l = std::make_shared<latch>(1);
		Watchdog<fake_clock> wd(100ms, [&]() { l->count_down(); });
		for (int i = 0; i < 5; i++) {
			REQUIRE_FALSE(l->wait_for(50ms));
			advanceFakeClock(50ms, 5ms, wd);
			REQUIRE_FALSE(l->wait_for(50ms));
			wd.feed();
		}
		REQUIRE_FALSE(l->wait_for(50ms));
		advanceFakeClock(200ms, 5ms, wd);
		REQUIRE(l->wait_for(50ms));

		// check that callback is not called again while starved
		l = std::make_shared<latch>(1);
		advanceFakeClock(200ms, 5ms, wd);
		REQUIRE_FALSE(l->wait_for(50ms));

		// check that callback is called after feeding and starving again
		wd.feed();
		REQUIRE_FALSE(l->wait_for(50ms));
		advanceFakeClock(100ms, 5ms, wd);
		REQUIRE(l->wait_for(50ms));
	}

	SECTION("Test keep calling while starved") {
		auto l = std::make_shared<latch>(1);
		Watchdog<fake_clock> wd(
			100ms, [&]() { l->count_down(); }, true);
		for (int i = 0; i < 5; i++) {
			REQUIRE_FALSE(l->wait_for(50ms));
			advanceFakeClock(50ms, 5ms, wd);
			REQUIRE_FALSE(l->wait_for(50ms));
			wd.feed();
		}
		REQUIRE_FALSE(l->wait_for(50ms));
		advanceFakeClock(100ms, 5ms, wd);
		REQUIRE(l->wait_for(50ms));

		// check that callback is repeatedly called while starved
		for (int i = 0; i < 3; i++) {
			l = std::make_shared<latch>(1);
			advanceFakeClock(100ms, 5ms, wd);
			REQUIRE(l->wait_for(50ms));
		}

		// check that callback is called after feeding and starving again
		l = std::make_shared<latch>(1);
		wd.feed();
		REQUIRE_FALSE(l->wait_for(50ms));
		advanceFakeClock(100ms, 5ms, wd);
		REQUIRE(l->wait_for(50ms));
	}
}

TEST_CASE("Test PeriodicTask", "[util][scheduler]") {
	// NOTE: after calling start() or stop() we must force the thread to suspend in order for
	// the PeriodicTask thread to start up and begin executing.
	// So it should be start() -> checkLatch() -> advanceTime() -> checkLatch().

	auto l = std::make_unique<latch>(1);
	PeriodicTask<fake_clock> pt(100ms, [&]() { l->count_down(); });

	// check that it doesn't start upon construction
	advanceFakeClock(350ms, 50ms, pt);
	REQUIRE_FALSE(l->wait_for(100ms));
	REQUIRE_FALSE(pt.isRunning());

	l = std::make_unique<latch>(3);
	pt.start();
	REQUIRE(pt.isRunning());
	REQUIRE_FALSE(l->wait_for(50ms));
	advanceFakeClock(350ms, 50ms, pt);
	REQUIRE(l->wait_for(50ms));

	pt.stop();
	REQUIRE_FALSE(pt.isRunning());
	l = std::make_unique<latch>(1);
	advanceFakeClock(150ms, 50ms, pt);
	REQUIRE_FALSE(l->wait_for(50ms));

	l = std::make_unique<latch>(3);
	pt.start();
	REQUIRE(pt.isRunning());
	REQUIRE_FALSE(l->wait_for(50ms));
	advanceFakeClock(350ms, 50ms, pt);
	REQUIRE(l->wait_for(50ms));
}

TEST_CASE("Test AsyncTask" "[util][scheduler]") {
	// since PeriodicTask uses AsyncTask, a lot of the functionality is already tested.
	SECTION("Test AsyncTask with task that terminates") {
		class ExitingAsyncTask : public AsyncTask<fake_clock> {
		public:
			ExitingAsyncTask(latch& l) : l_(l) {}
		protected:
			void task(std::unique_lock<std::mutex>& lock) override {
				wait_for(lock, 100ms);
				l_.count_down();
			}
		private:
			latch& l_;
		};

		latch l(1);
		ExitingAsyncTask task(l);
		REQUIRE_FALSE(task.isRunning());
		task.start();
		REQUIRE(task.isRunning());
		REQUIRE_FALSE(l.wait_for(50ms));
		AsyncTask<fake_clock>& at = task;
		advanceFakeClock(150ms, 50ms, at);
		REQUIRE(l.wait_for(50ms));
	}

	SECTION("Test that AsyncTask stops when killed during wait") {
		class WaitingAsyncTask : public AsyncTask<fake_clock> {
		public:
			WaitingAsyncTask(latch& l) : l_(l) {}
		protected:
			void task(std::unique_lock<std::mutex>& lock) override {
				wait_for(lock, 100ms);
				l_.count_down();
			}
		private:
			latch& l_;
		};

		latch l(1);
		WaitingAsyncTask task(l);
		REQUIRE_FALSE(task.isRunning());
		task.start();
		REQUIRE(task.isRunning());
		REQUIRE_FALSE(l.wait_for(50ms));
		AsyncTask<fake_clock>& at = task;
		advanceFakeClock(50ms, 50ms, at);
		REQUIRE_FALSE(l.wait_for(50ms));
		task.stop();
		REQUIRE(l.wait_for(50ms));
	}
}
