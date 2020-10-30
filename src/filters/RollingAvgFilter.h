#pragma once

#include <Eigen/Core>
#include <vector>

/**
 * Implements a rolling average filter of the specified type. Ideally this would use a template
 * but that causes weird issues with averaging for some reason.
 */
class RollingAvgFilter
{
public:
	explicit RollingAvgFilter(int size)
		: size(size), dataPoints(std::vector<Eigen::Vector3d>())
	{
	}

	/**
	 * Get the output of the filter.
	 *
	 * @return The output of the filter. If no data has been entered, returns the default
	 * value.
	 */
	Eigen::Vector3d get() const
	{
		Eigen::Vector3d sum;
		if (dataPoints.empty())
		{
			return sum;
		}
		for (const Eigen::Vector3d &t : dataPoints)
		{
			sum += t;
		}
		return sum / dataPoints.size();
	}

	/**
	 * Adds data to the filter and gets the new output.
	 *
	 * @param data The data to add to the filter.
	 * @return The new output of the filter after adding this data.
	 */
	Eigen::Vector3d get(const Eigen::Vector3d &data)
	{
		dataPoints.push_back(data);
		while (dataPoints.size() > size)
		{
			dataPoints.erase(dataPoints.begin());
		}

		return get();
	}

	void reset()
	{
		dataPoints.clear();
	}

	int getNumPoints() const
	{
		return dataPoints.size();
	}

	int getSize() const
	{
		return size;
	}

	std::vector<Eigen::Vector3d> getData() const
	{
		return dataPoints;
	}

private:
	std::vector<Eigen::Vector3d> dataPoints;
	int size;
};
