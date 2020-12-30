#pragma once

#include <Eigen/Core>
#include <vector>

/**
 * Implements a rolling average filter of the specified type. Can only use double vectors.
 */
template <int numPoints, int numDims> class RollingAvgFilter
{
public:
	/**
	 * Get the output of the filter.
	 *
	 * @return The output of the filter. If no data has been entered, returns the default
	 * value.
	 */
	Eigen::Matrix<double, numDims, 1> get() const
	{
		Eigen::Matrix<double, numDims, 1> ret = Eigen::Matrix<double, numDims, 1>::Zero();
		if (size == 0)
		{
			return ret;
		}
		int numData = std::min(numPoints, size);
		for (int i = 0; i < numData; i++)
		{
			ret += data.col(i);
		}
		return ret / numData;
	}

	/**
	 * Adds data to the filter and gets the new output.
	 *
	 * @param val The data to add to the filter.
	 * @return The new output of the filter after adding this data.
	 */
	Eigen::Matrix<double, numDims, 1> get(const Eigen::Matrix<double, numDims, 1> &val)
	{
		data.col(index) = val;
		index = (index + 1) % numPoints;
		if (size < numPoints)
		{
			size++;
		}

		return get();
	}

	/**
	 * Clears all data in the buffer.
	 */
	void reset()
	{
		size = 0;
		index = 0;
	}

	/**
	 * Returns the number of points stored in the buffer.
	 *
	 * @return The number of points stored in the buffer, in the range [0, numPoints]
	 */
	int getSize() const
	{
		return size;
	}

	/**
	 * Returns the maximum number of points that can be stored in the buffer.
	 *
	 * @return The maximum number of points that can be stored in the buffer.
	 */
	int getBufferSize() const
	{
		return numPoints;
	}

	/**
	 * Get the underlying buffer of the filter.
	 *
	 * @return The Matrix representing the buffer of the filter. Data points are stored in each
	 * column.
	 */
	Eigen::Matrix<double, numDims, numPoints> getData() const
	{
		return data;
	}

private:
	Eigen::Matrix<double, numDims, numPoints> data; // ring buffer, starts with garbage data
	int size = 0;
	int index = 0;
};
