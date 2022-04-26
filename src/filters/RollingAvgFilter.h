#pragma once

#include <iterator>
#include <list>
#include <numeric>

namespace filters {

/**
 * @brief Implements a rolling average filter of the specified type.
 *
 * @tparam T The data type to filter. Must support commutative addition as well as scalar
 * division.
 */
template <typename T> class RollingAvgFilter {
public:
	/**
	 * @brief Construct a new rolling average filter.
	 *
	 * @param numPoints The maximum number of points that can be stored in the buffer.
	 */
	explicit RollingAvgFilter(size_t numPoints)
		: numPoints(numPoints), data(), dataIter(data.begin()) {}

	/**
	 * @brief Get the output of the filter.
	 *
	 * Behavior is undefined if no data is in the filter.
	 *
	 * @return The output of the filter.
	 *
	 * @warning If no data is in the filter the behavior is undefined.
	 */
	T get() const {
		return std::reduce(std::next(data.begin()), data.end(), data.front()) / data.size();
	}

	/**
	 * @brief Adds data to the filter and gets the new output.
	 *
	 * @param val The data to add to the filter.
	 * @return The new output of the filter after adding this data.
	 */
	T get(const T& val) {
		if (data.size() < numPoints) {
			data.push_back(val);
			dataIter = data.begin();
		} else {
			*dataIter = val;
			if (++dataIter == data.end()) {
				dataIter = data.begin();
			}
		}

		return get();
	}

	/**
	 * @brief Clears all data in the buffer.
	 */
	void reset() {
		data.clear();
		dataIter = data.begin();
	}

	/**
	 * @brief Returns the number of points stored in the buffer.
	 *
	 * @return The number of points stored in the buffer, in the range [0, numPoints]
	 */
	int getSize() const {
		return data.size();
	}

	/**
	 * @brief Get the maximum number of points that can be stored in the buffer.
	 *
	 * @return The maximum number of points that can be stored in the buffer.
	 */
	int getBufferSize() const {
		return numPoints;
	}

	/**
	 * @brief Get the underlying buffer of the filter.
	 *
	 * @return A copy of the underlying buffer.
	 */
	std::list<T> getData() const {
		return data;
	}

private:
	size_t numPoints;
	std::list<T> data;
	typename std::list<T>::iterator dataIter;
};

} // namespace filters
