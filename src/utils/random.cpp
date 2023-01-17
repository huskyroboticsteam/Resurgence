#include "random.h"

#include <chrono>
#include <random>

namespace util {

/**
 * There still might be some variation due to the dependence of the robot position
 * on when exactly each context switch occurs.
 *
 * For programs with more threads, a more sophisticated solution will be necessary.
 */
static std::normal_distribution<double> stdn_dist(0.0, 1.0);
static long seed = std::chrono::system_clock::now().time_since_epoch().count();
// long seed = 1626474823108702150;
static std::default_random_engine main_generator(seed);
static std::default_random_engine spin_generator(seed);

long getNormalSeed() {
	return seed;
}

double stdn(int thread_id) {
	return stdn_dist(thread_id == 0 ? main_generator : spin_generator);
}

uint64_t getUnixTime() {
	return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
									 std::chrono::system_clock::now().time_since_epoch())
									 .count());
}

} // namespace util
