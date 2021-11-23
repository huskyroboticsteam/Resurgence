#pragma once

#include "../simulator/utils.h"

#include <chrono>

// the clock used for time measurements for data
using dataclock = std::chrono::steady_clock;
// a point in time as measured by dataclock
using datatime_t = std::chrono::time_point<dataclock>;

/**
 * @brief Represents data measured using a sensor at a given time.
 *
 * @tparam T The type of data measured from the sensor. Requires defined default constructor.
 */
template <typename T> class DataPoint {
public:
	/**
	 * @brief Construct an invalid DataPoint, holding no data
	 */
	DataPoint() : valid(false), time(), data() {}

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
	DataPoint(datatime_t time, T data) : valid(true), time(time), data(data) {}

	// provide an implicit conversion to the data type. Helps with backwards compatability.
	operator T() const { return data; }

	// Check if this measurement is valid
	operator bool() const { return valid; }

	/**
	 * @brief Check if this measurement was taken recently.
	 *
	 * @param duration The data is fresh if it was taken at most this many milliseconds ago.
	 * @return true if the data is valid and was measured at most \p duration milliseconds ago,
	 * false otherwise.
	 */
	bool isFresh(std::chrono::milliseconds duration) {
		return valid && dataclock::now() - duration <= time;
	}

	// Check if this measurement is valid
	bool isValid() { return valid; }

	// The time at which the data was measured. Use only if data is valid.
	datatime_t getTime() { return time; }

	// The measurement data. Use only if data is valid.
	T getData() { return data; }

private:
	// true iff this measurement is valid, false otherwise.
	bool valid;
	// the time at which the data was measured. Use only if valid == true.
	datatime_t time;
	// the measurement data. Use only if valid == true.
	T data;
};

// Call this before trying to do anything else
void world_interface_init();

// If the requested dtheta/dx is too fast for the robot to execute, it will
// scale them down and return the corresponding scale factor.
// TODO: indicate what the return value of setCmdVel() means
double setCmdVel(double dtheta, double dx);

std::pair<double, double> getCmdVel();

// read measurement from the lidar
DataPoint<points_t> readLidarScan();

/**
 * @brief Read measurement from the CV system. As of now, returns a vector of fixed length, one
 * for each post in the competition.
 *
 * @return DataPoint<points_t> A vector of fixed lengths. Non-visible markers are denoted with
 * {0,0,0}, while all nonzero points are visible marker. The index of a landmark in this vector
 * is its id.
 */
DataPoint<points_t> readLandmarks();

// Get the current transform in the global frame based on a GPS measurement.
// Note that these values are NOT lat/long.
// TODO: figure out differential correction and adjust API accordingly
DataPoint<transform_t> readGPS();

// Calculates the current transform in the global frame based purely on forward kinematics
DataPoint<transform_t> readOdom();

// Calculate the current pose velocity (in the robot's reference frame) using visual odometry.
DataPoint<pose_t> readVisualOdomVel();

// Given the current longitude and latitude, convert to a point_t position representation
point_t gpsToMeters(double lon, double lat);

// `index` must be in the range 0-6 (the URC competition will have 7 legs)
URCLeg getLeg(int index);

/**
 * @brief Set the PWM command of the given motor.
 * 
 * @param motor The name of the motor. Must be a valid motor name.
 * @param normalizedPWM The PWM command, in the range [-1, 1]
 */
void setMotorPWM(const std::string &motor, double normalizedPWM);

/**
 * @brief Set the target position of the motor.
 * 
 * @param motor The name of the motor. Must be a valid motor name.
 * @param targetPos The target position. Refer to the specific motor for more information.
 */
void setMotorPos(const std::string &motor, int32_t targetPos);
