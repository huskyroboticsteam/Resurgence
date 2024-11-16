#include "../../src/utils/threading.h"

#include <chrono>
#include <thread>

#include <catch2/catch.hpp>

using namespace util;

using namespace std::chrono_literals;

TEST_CASE("Test Wait", "[util][threading]") {
	SECTION("Wait until latch unlocks when countdown reaches zero") {
		latch l(3);
		std::thread worker([&]() {
			l.count_down();
			l.count_down();
			l.count_down();
		});

		l.wait();

		worker.join();

		REQUIRE(true); // if thread isn't blocked, will be reached
	}
}

TEST_CASE("Test Wait For", "[util][threading]") {
	SECTION("Wait for latch unlocks within timeout") {
		latch l(1);
		std::thread worker([&]() {
			std::this_thread::sleep_for(50ms);
			l.count_down();
		});

		REQUIRE(l.wait_for(100ms));

		worker.join();
	}

	SECTION("Wait for latch times out when not unlocked") {
		latch l(1);

		REQUIRE_FALSE(l.wait_for(50ms));
	}
}

TEST_CASE("Test Wait Until", "[util][threading]") {
	SECTION("Wait until latch unlocks before timeout") {
		latch l(1);
		std::thread worker([&]() {
			std::this_thread::sleep_for(50ms);
			l.count_down();
		});

		auto timeout = std::chrono::steady_clock::now() + 100ms;
		REQUIRE(l.wait_until(timeout));

		worker.join();
	}

	SECTION("Wait until latch times out") {
		latch l(1);
		auto timeout = std::chrono::steady_clock::now() + 50ms;
		REQUIRE_FALSE(l.wait_until(timeout));
	}
}

TEST_CASE("Test Count Down", "[util][threading]") {
	SECTION("Latch unlocks after enough countdowns") {
		latch l(3);

		REQUIRE_FALSE(l.wait_for(10ms));

		l.count_down();
		l.count_down();
		REQUIRE_FALSE(l.wait_for(10ms));

		l.count_down();
		REQUIRE(l.wait_for(10ms));
	}

	SECTION("Latch remains locked if not counted down enough") {
		latch l(3);

		l.count_down();
		l.count_down();
		REQUIRE_FALSE(l.wait_for(10ms));
	}

	SECTION("Latch can count down by more than 1") {
		latch l(3);

		l.count_down(2);
		REQUIRE_FALSE(l.wait_for(10ms));

		l.count_down(1);
		REQUIRE(l.wait_for(10ms));
	}
}