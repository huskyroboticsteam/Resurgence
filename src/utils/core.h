#pragma once

#include <algorithm>
#include <functional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

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
template <typename T>
std::string to_string(const T& val) {
	return std::to_string(val);
}

/**
 * @brief Converts a boolean to a string.
 *
 * @param val The value to get the string representation of.
 * @return "true" iff val, otherwise "false".
 */
template <>
std::string to_string<bool>(const bool& val);

/**
 * @brief Convert the given std::string to a frozen::string.
 */
frozen::string freezeStr(const std::string& str);

/**
 * @brief Converts a pair to a tuple. Elements are copied to the returned tuple.
 *
 * @tparam T The type of the first element.
 * @tparam U The type of the second element.
 * @param pair The pair to convert to a tuple.
 * @return std::tuple<T, U> The converted tuple.
 */
template <typename T, typename U>
std::tuple<T, U> pairToTuple(const std::pair<T, U>& pair) {
	return std::tuple<T, U>(pair.first, pair.second);
}

/**
 * @brief A helper class for executing a function when leaving a scope, in RAII-style.
 */
class RAIIHelper {
public:
	/**
	 * @brief Construct a new RAIIHelper.
	 *
	 * @param f The function to execute when this object is destructed.
	 */
	RAIIHelper(const std::function<void()>& f);

	RAIIHelper(const RAIIHelper&) = delete;

	/**
	 * @brief Move an RAIIHelper into another.
	 *
	 * The RAIIHelper being moved is guaranteed to be empty after this, i.e. will not execute
	 * any code when being destructed.
	 *
	 * @param other The RAIIHelper to move.
	 */
	RAIIHelper(RAIIHelper&& other);

	RAIIHelper& operator=(const RAIIHelper&) = delete;

	/**
	 * @brief Destroy the RAIIHelper object, executing the given function, if not empty.
	 */
	~RAIIHelper();

private:
	std::function<void()> f;
};

} // namespace util
