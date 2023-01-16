#pragma once

namespace util {

/**
 * @brief Sample from the standard normal distribution.
 *
 * The thread_id is used to choose which random number generator to use.
 * This is important when trying to rerun a particular random seed;
 * each thread needs its own dedicated sequence of random numbers.
 *
 * @param thread_id Either 0 or 1, denoting which random number generator use.
 * @return double
 */
double stdn(int thread_id);

/**
 * @brief Get the random seed used for util::stdn().
 *
 * @return The random seed.
 */
long getNormalSeed();

} // namespace util
