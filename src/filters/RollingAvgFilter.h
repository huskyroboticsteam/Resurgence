#pragma once


#include <vector>
#include <Eigen/Core>

/**
 * Implements a rolling average filter of the specified type. Ideally this would use a template but that causes
 * weird issues with averaging for some reason.
 */
class RollingAvgFilter {
public:
    explicit RollingAvgFilter(int size) : size(size), dataPoints(std::vector<Eigen::Vector3d>()) {}

    /**
     * Get the output of the filter.
     *
     * @return The output of the filter. If no data has been entered, returns the default value.
     */
    Eigen::Vector3d get() {
        Eigen::Vector3d sum;
        if (dataPoints.size() == 0) {
            return sum;
        }
        for (Eigen::Vector3d t : dataPoints) {
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
    Eigen::Vector3d get(const Eigen::Vector3d &data) {
        dataPoints.push_back(data);
        while (dataPoints.size() > size) {
            dataPoints.erase(dataPoints.begin());
        }

        return get();
    }

    int getNumPoints() {
        return dataPoints.size();
    }

    int getSize() {
        return size;
    }

    std::vector<Eigen::Vector3d> getData() {
        return dataPoints;
    }

private:
    std::vector<Eigen::Vector3d> dataPoints;
    int size;
};
