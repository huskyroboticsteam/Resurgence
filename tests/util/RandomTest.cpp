#include "../../src/utils/random.h"

#include <catch2/catch.hpp>

using namespace util;

TEST_CASE("Test Standard Normal Distribution w/ different thread_ids", "[util][random]") {
	double sample0 = stdn(0);
	double sample1 = stdn(1);

	// samples should be different
	REQUIRE(sample0 != sample1);
}

TEST_CASE("Test Get Normal Seed", "[util][random]") {
	double seed1 = getNormalSeed();
	double seed2 = getNormalSeed();

	// seeds should be the same
	REQUIRE(seed1 == seed2);
}
