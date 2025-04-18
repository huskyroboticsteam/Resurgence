#pragma once

#include "../navtypes.h"

#include <bitset>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include <frozen/string.h>
#include <frozen/unordered_map.h>
#include <frozen/unordered_set.h>

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

/** @brief The clock used for time measurements for data */
using dataclock = std::chrono::steady_clock;
/** @brief A point in time as measured by dataclock */
using datatime_t = std::chrono::time_point<dataclock>;

template <typename T>
class DataPoint;

/**
 * @brief A data structure that represents when each landmark was seen.
 *
 * @see AR::readLandmarks()
 */
using landmarks_t = std::vector<DataPoint<navtypes::point_t>>;

/** @brief A pair of a camera frame and its corresponding frame number */
using CameraFrame = std::pair<cv::Mat, uint32_t>;
/** @brief The type of a camera id */
using CameraID = std::string;

/** @brief An indication enum, used to command the LED to flash different signals */
enum class indication_t {
	off,
	autonomous,
	teleop,
	arrivedAtDest
};

/** @brief The motors on the robot. */
enum class motorid_t {
	leftTread,
	rightTread,
	armBase,
	shoulder,
	elbow,
	forearm,
	wristDiffRight,
	wristDiffLeft,
	hand,
	activeSuspension,
	drillActuator,
	drillMotor
};

/** @brief the mounted peripheral on the robot. */
enum class mountedperipheral_t {
	none,
	arm,
	armServo,
	scienceStation,
	lidar
};

enum class jointid_t {
	armBase,
	shoulder,
	elbow,
	forearm,
	wristPitch,
	wristRoll,
	hand,
	activeSuspension,
	ikForward,
	ikUp
};

constexpr auto all_jointid_t = frozen::make_unordered_set<jointid_t>(
	{jointid_t::armBase, jointid_t::shoulder, jointid_t::elbow, jointid_t::forearm,
	 jointid_t::wristRoll, jointid_t::wristPitch, jointid_t::hand, jointid_t::activeSuspension,
	 jointid_t::ikForward, jointid_t::ikUp});

constexpr auto name_to_jointid = frozen::make_unordered_map<frozen::string, jointid_t>(
	{{"armBase", jointid_t::armBase},
	 {"shoulder", jointid_t::shoulder},
	 {"elbow", jointid_t::elbow},
	 {"forearm", jointid_t::forearm},
	 {"wristPitch", jointid_t::wristPitch},
	 {"wristRoll", jointid_t::wristRoll},
	 {"hand", jointid_t::hand},
	 {"activeSuspension", jointid_t::activeSuspension},
	 {"ikForward", jointid_t::ikForward},
	 {"ikUp", jointid_t::ikUp}});

class bad_datapoint_access : public std::runtime_error {
public:
	bad_datapoint_access() : std::runtime_error("bad_datapoint_access") {}
};

/**
 * @brief Represents data measured using a sensor at a given time.
 *
 * @tparam T The type of data measured from the sensor.
 */
template <typename T>
class DataPoint {
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
	 *
	 * @return T The value of this data point.
	 * @throw bad_datapoint_access If this data point is not valid.
	 */
	T getData() const {
		if (isValid()) {
			return datapoint.value().second;
		} else {
			throw bad_datapoint_access();
		}
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

	/**
	 * @brief Transforms the data in this datapoint by the given function.
	 *
	 * @tparam F A callable type that accepts @p T and outputs some other type.
	 * @param f The function that transforms data.
	 * @return DataPoint<std::invoke_result_t<F, T>> A new datapoint that holds
	 * the output of `f(getData())` and has the same timestamp as this datapoint
	 * if it's valid, otherwise an empty datapoint.
	 */
	template <typename F>
	DataPoint<std::invoke_result_t<F, T>> transform(const F& f) {
		if (isValid()) {
			return DataPoint<std::invoke_result_t<F, T>>(getTime(), f(getData()));
		} else {
			return {};
		}
	}

private:
	/**
	 * @brief The time and measurement data.
	 */
	std::optional<std::pair<datatime_t, T>> datapoint;
};

/**
 * @brief The maximum number of limit switches associated with any motor.
 */
constexpr size_t N_LIMIT_SWITCH = 8;

constexpr int LIMIT_SWITCH_LIM_MIN_IDX = 0;
constexpr int LIMIT_SWITCH_LIM_MAX_IDX = 1;

/**
 * @brief A class that represents limit switches from a specific motor.
 */
class LimitSwitchData {
public:
	/**
	 * @brief Construct a new LimitSwitchData object, from the given bits.
	 *
	 * A 1 bit means closed, and 0 means open.
	 * Only the rightmost N_LIMIT_SWITCH bits are used.
	 *
	 * @param data The limit switch data.
	 */
	LimitSwitchData(unsigned long long data);

	/**
	 * @brief Check if the given index is open.
	 *
	 * This method is the opposite of LimitSwitchData::isClosed().
	 *
	 * @param idx The index to check. 0 <= idx < N_LIMIT_SWITCH.
	 * @return bool True iff the switch at the given index is open.
	 */
	bool isOpen(size_t idx);

	/**
	 * @brief Check if the given index is closed.
	 *
	 * This method is the opposite of LimitSwitchData::isOpen().
	 *
	 * @param idx The index to check. 0 <= idx < N_LIMIT_SWITCH.
	 * @return bool True iff the switch at the given index is closed.
	 */
	bool isClosed(size_t idx);

	/**
	 * @brief Check if any index is open.
	 *
	 * @return bool True iff any index is open.
	 */
	bool isAnyOpen();

	/**
	 * @brief Check if any index is closed.
	 *
	 * @return bool True iff any index is closed.
	 */
	bool isAnyClosed();

	/**
	 * @brief Check which indices differ between this data and other.
	 *
	 * This is useful to see which indices have recently changed.
	 *
	 * @param other The data to check against.
	 * @return std::bitset<N_LIMIT_SWITCH> A bitset where an index is 1 if
	 * that this and other differ at that index, and 0 otherwise.
	 */
	std::bitset<N_LIMIT_SWITCH> diff(const LimitSwitchData& other);

private:
	std::bitset<N_LIMIT_SWITCH> data;
};

} // namespace robot::types

namespace util {
std::string to_string(robot::types::jointid_t joint);
std::string to_string(const robot::types::CameraID& id);
std::string to_string(robot::types::mountedperipheral_t peripheral);
} // namespace util
