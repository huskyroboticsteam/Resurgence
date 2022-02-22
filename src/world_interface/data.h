#pragma once

#include "../navtypes.h"

#include <chrono>
#include <optional>
#include <vector>

// forward declare cv::Mat instead of importing OpenCV
// we do this to avoid unnecessarily including OpenCV in all build targets
namespace cv {
class Mat;
}

/**
 * @namespace robot::types
 * @brief Types for use in the world interface.
 */
namespace robot::types {

// the clock used for time measurements for data
using dataclock = std::chrono::steady_clock;
// a point in time as measured by dataclock
using datatime_t = std::chrono::time_point<dataclock>;

template <typename T> class DataPoint;
using landmarks_t = std::vector<DataPoint<navtypes::point_t>>;

// A pair of a camera frame and its corresponding frame number
using CameraFrame = std::pair<cv::Mat, uint32_t>;
// The type of a camera id
using CameraID = std::string;

// An indication enum, used to command the LED to flash different signals
enum class indication_t { off, autonomous, teleop, arrivedAtDest };

/**
 * @brief Represents data measured using a sensor at a given time.
 *
 * @tparam T The type of data measured from the sensor.
 */
template <typename T> class DataPoint {
public:
	/**
	 * @brief Construct an invalid DataPoint, holding no data
	 */
	DataPoint() : datapoint() {}

	/**
	 * @brief Construct a new DataPoint object, measured now.
	 *
	 * @param data The piece of data
	 */
	DataPoint(T data) : DataPoint(dataclock::now(), data) {}

	/**
	 * @brief Construct a new DataPoint object, measured at the given time.
	 *
	 * @param time The time at which the data was measured.
	 * @param data The piece of data.
	 */
	DataPoint(datatime_t time, T data) : datapoint({time, data}) {}

	// provide an implicit conversion to the data type. Helps with backwards compatability.
	operator T() const {
		return getData();
	}

	// Check if this measurement is valid
	operator bool() const {
		return isValid();
	}

	/**
	 * @brief Check if this measurement was taken recently.
	 *
	 * @param duration The data is fresh if it was taken at most this many milliseconds ago.
	 * @return true if the data is valid and was measured at most \p duration milliseconds ago,
	 * false otherwise.
	 */
	bool isFresh(std::chrono::milliseconds duration) const {
		return isValid() && dataclock::now() - duration <= getTime();
	}

	// Check if this measurement is valid
	bool isValid() const {
		return datapoint.has_value();
	}

	// The time at which the data was measured. Use only if data is valid.
	datatime_t getTime() const {
		return datapoint.value().first;
	}

	// The measurement data. Use only if data is valid.
	T getData() const {
		return datapoint.value().second;
	}

private:
	// the time and measurement data
	std::optional<std::pair<datatime_t, T>> datapoint;
};

} // namespace robot::types
