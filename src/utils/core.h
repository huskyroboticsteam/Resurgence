#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include <frozen/string.h>

namespace util {

/**
 * @brief Check if two numbers are approximately equal.
 *
 * @param a The first number.
 * @param b The second number.
 * @param threshold If @c |a-b|&ge;threshold then they are not approximately equal.
 * @return true iff @c |a-b|<threshold.
 */
bool almostEqual(double a, double b, double threshold = 1e-6);

/**
 * @brief Get the keys of the given map.
 *
 * @param An unordered map.
 * @return The keys of the given map, as an unordered set.
 */
template <typename K, typename V>
std::unordered_set<K> keySet(const std::unordered_map<K, V>& input) {
	std::unordered_set<K> output;
	std::transform(input.begin(), input.end(), std::inserter(output, output.end()),
				   [](auto pair) { return pair.first; });
	return output;
}

/**
 * @brief Convert the given value to a string.
 *
 * This method is necessary because we cannot extend the std namespace; having our own method
 * allows us to extend it with template specializations whenever we want, and have it "fall
 * back" to using the std version when no specialization is available.
 *
 * @param val The value to convert to string.
 * @tparam The type of the value to convert to string.
 * @return A string representation of that value, as a std::string. The exact representation of
 * the value is up to the implementation.
 */
template <typename T> std::string to_string(const T& val) {
	return std::to_string(val);
}

/**
 * @brief Convert the given std::string to a frozen::string.
 */
frozen::string freezeStr(const std::string& str);

} // namespace util