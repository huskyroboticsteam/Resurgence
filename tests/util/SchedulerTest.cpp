#include "../../src/utils/scheduler.h"

#include "../../src/utils/latch.h"

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

void advanceFakeClock(std::chrono::milliseconds dur, std::chrono::milliseconds inc,
					  PeriodicScheduler<fake_clock>& scheduler) {
	for (int i = 0; i < dur / inc; i++) {
		fake_clock::advance(inc);
		util::impl::notifyScheduler(scheduler);
	}
}
} // namespace

TEST_CASE("Test PeriodicScheduler", "[util]") {
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
