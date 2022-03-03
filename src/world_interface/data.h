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

enum class motorid_t {
	frontLeftWheel,
	frontRightWheel,
	rearLeftwheel,
	rearRightWheel,
	armBase,
	shoulder,
	elbow,
	forearm,
	differentialRight,
	differentialLeft,
	hand
};

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

	/**
	 * @brief Implicitly convert to the required data type. UB if datapoint is invalid.
	 * Equivalent to getData().
	 *
	 * @return T The value of this datapoint.
	 */
	operator T() const {
		return getData();
	}

	/**
	 * @brief Check if this measurement is valid. Equivalent to isValid().
	 *
	 * @return bool True iff this datapoint has a value.
	 */
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

	/**
	 * @brief Check if this measurement is valid.
	 *
	 * @return bool True iff this data point has a value.
	 */
	bool isValid() const {
		return datapoint.has_value();
	}

	/**
	 * @brief Get the time at which the measurement was taken.
	 * UB if data point is not valid.
	 *
	 * @return datatime_t The time at which the measurement was taken.
	 */
	datatime_t getTime() const {
		return datapoint.value().first;
	}

	/**
	 * @brief Get the value of this data point.
	 * UB if data point is not valid.
	 *
	 * @return T The value of this data point.
	 */
	T getData() const {
		return datapoint.value().second;
	}

	/**
	 * @brief Get the value of this data point, defaulting to a given value
	 * if this data point is invalid.
	 *
	 * @param defaultData The value to return if this data point is not valid.
	 * @return T The value of this data point, or @p defaultData.
	 */
	T getDataOrElse(T defaultData) {
		return isValid() ? getData() : defaultData;
	}

private:
	/**
	 * @brief The time and measurement data.
	 */
	std::optional<std::pair<datatime_t, T>> datapoint;
};

} // namespace robot::types
