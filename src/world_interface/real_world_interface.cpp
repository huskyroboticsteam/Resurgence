#include "../Globals.h"
#include "../Networking/CANUtils.h"
#include "../Networking/json.hpp"
#include "../Networking/motor_interface.h"
#include "../Networking/ParseCAN.h"
#include "../Util.h"
#include "../ar/read_landmarks.h"
#include "../camera/Camera.h"
#include "../gps/read_usb_gps.h"
#include "../lidar/read_hokuyo_lidar.h"
#include "../log.h"
#include "../simulator/utils.h"
#include "world_interface.h"

#include <future>
#include <iostream>
#include <set>

#include <opencv2/calib3d.hpp>

extern "C" {
#include "../HindsightCAN/CANMotorUnit.h"
}

using nlohmann::json;

struct timeval last_odom_reading_;
transform_t last_odom_tf_;
double odom_dtheta_, odom_dx_;

void world_interface_init() {
	bool gps_success = startGPSThread();
	bool lidar_success = lidar::initializeLidar();
	bool landmark_success = AR::initializeLandmarkDetection();

	gettimeofday(&last_odom_reading_, NULL);
	last_odom_tf_ = toTransform({0., 0., 0.});
	odom_dtheta_ = 0.0;
	odom_dx_ = 0.0;
}

transform_t getOdomAt(const struct timeval& time) {
	double elapsed = getElapsedUsecs(last_odom_reading_, time) / 1000000.0;
	double delta_theta = odom_dtheta_ * elapsed;
	double rel_x, rel_y, rel_th;
	rel_th = odom_dtheta_ * elapsed;
	if (fabs(odom_dtheta_) < 0.000001) {
		rel_x = odom_dx_ * elapsed;
		rel_y = 0.0;
	} else {
		double turn_radius = odom_dx_ / odom_dtheta_;
		rel_x = turn_radius * sin(rel_th);
		rel_y = turn_radius * (1 - cos(rel_th));
	}
	transform_t rel_tf = toTransform({rel_x, rel_y, rel_th});
	return rel_tf * last_odom_tf_;
}

void setCmdVelToIntegrate(double dtheta, double dx) {
	struct timeval now;
	gettimeofday(&now, NULL);
	transform_t new_tf = getOdomAt(now);
	last_odom_reading_ = now;
	last_odom_tf_ = new_tf;
	odom_dtheta_ = dtheta;
	odom_dx_ = dx;
}

// Distance between left and right wheels.
constexpr double WHEEL_BASE = 0.66;
// Effective distance between wheels. Tweaked so that actual rover angular rate
// roughly matches the commanded angular rate.
constexpr double EFF_WHEEL_BASE = 1.40;

constexpr double WHEEL_RADIUS = 0.15;		  // Eyeballed
constexpr double PWM_FOR_1RAD_PER_SEC = 5000; // Eyeballed
// This is a bit on the conservative side, but we heard an ominous popping sound at 20000.
constexpr double MAX_PWM = 20000;

double setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0))
		return false;

	/* This is the inverse of the formula:
	 *		dx = (right_ground_vel + left_ground_vel) / 2
	 *		dtheta = (right_ground_vel - left_ground_vel) / EFF_WHEEL_BASE
	 */
	double right_ground_vel = dx + EFF_WHEEL_BASE / 2 * dtheta;
	double left_ground_vel = dx - EFF_WHEEL_BASE / 2 * dtheta;
	double right_angular_vel = right_ground_vel / WHEEL_RADIUS;
	double left_angular_vel = left_ground_vel / WHEEL_RADIUS;
	int16_t right_pwm = (int16_t)(right_angular_vel * PWM_FOR_1RAD_PER_SEC);
	int16_t left_pwm = (int16_t)(left_angular_vel * PWM_FOR_1RAD_PER_SEC);
	log(LOG_TRACE, "dtheta %f dx %f right ground %f right angular %f right pwm %d\n", dtheta,
		dx, right_ground_vel, right_angular_vel, right_pwm);
	double scale_down_factor = 1.0;
	if (abs(right_pwm) > MAX_PWM) {
		log(LOG_WARN, "WARNING: requested too-large right PWM %d\n", right_pwm);
		scale_down_factor = abs(right_pwm) / MAX_PWM;
	}
	if (abs(left_pwm) > MAX_PWM) {
		log(LOG_WARN, "WARNING: requested too-large left PWM %d\n", left_pwm);
		double scale_down_factor_left = abs(left_pwm) / MAX_PWM;
		if (scale_down_factor_left > scale_down_factor)
			scale_down_factor = scale_down_factor_left;
	}
	right_pwm = (int16_t)(right_pwm / scale_down_factor);
	left_pwm = (int16_t)(left_pwm / scale_down_factor);
	if (scale_down_factor < 1.0) {
		log(LOG_WARN, "Scaling down cmd_vel by %f to %f %f\n", scale_down_factor,
			dtheta / scale_down_factor, dx / scale_down_factor);
	}

	setCmdVelToIntegrate(dtheta / scale_down_factor, dx / scale_down_factor);

	CANPacket p;
	uint8_t motor_group = 0x04;
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_FL, left_pwm);
	sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_FR, right_pwm);
	sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BL, left_pwm);
	sendCANPacket(p);
	AssemblePWMDirSetPacket(&p, motor_group, DEVICE_SERIAL_MOTOR_CHASSIS_BR, right_pwm);
	sendCANPacket(p);

	return scale_down_factor;
}

std::pair<double, double> getCmdVel() {
    return {odom_dtheta_, odom_dx_};
}

DataPoint<points_t> readLandmarks() {
	return AR::readLandmarks();
}

DataPoint<points_t> readLidarScan() {
	if (lidar::isLidarDataFresh()) {
		return lidar::readLidar();
	} else {
		return points_t{};
	}
}

DataPoint<transform_t> readOdom() {
	struct timeval now;
	gettimeofday(&now, NULL);
	return getOdomAt(now);
}

DataPoint<pose_t> readVisualOdomVel() {
    return DataPoint<pose_t>{};
}

URCLeg getLeg(int index) {
	return URCLeg{0, -1, {0., 0., 0.}};
}

const std::map<std::string, double> positive_arm_pwm_scales = {
	{"arm_base",   12000},
	{"shoulder",  -20000},
	{"elbow",     -32768},
	{"forearm",    -6000},
	{"diffleft",    5000},
	{"diffright",  -5000},
	{"hand",       15000}};
const std::map<std::string, double> negative_arm_pwm_scales = {
	{"arm_base", 12000},
	{"shoulder", -14000},
	{"elbow", -18000},
	{"forearm", -6000},
	{"diffleft", 5000},
	{"diffright", -10000},
	{"hand", 15000}};
const std::map<std::string, double> incremental_pid_scales = {
	{"arm_base", M_PI / 8}, // TODO: Check signs
	{"shoulder", -M_PI / 8},
	{"elbow", -M_PI / 8},
	{"forearm", 0}, // We haven't implemented PID on these motors yet
	{"diffleft", 0},
	{"diffright", 0},
	{"hand", 0}};


template <typename T> int getIndex(const std::vector<T> &vec, const T &val) {
    auto itr = std::find(vec.begin(), vec.end(), val);
    return itr == vec.end() ? -1 : itr - vec.begin();
}

void setMotorPWM(const std::string &motor, double normalizedPWM) {
    CANPacket p;
	auto& scale_map = (normalizedPWM > 0 ? positive_arm_pwm_scales : negative_arm_pwm_scales);
	double pwm = normalizedPWM * scale_map.at(motor);
	int motor_serial = getIndex(motor_group, motor);
	AssemblePWMDirSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, pwm);
	sendCANPacket(p);
}

void setMotorPos(const std::string &motor, int32_t targetPos) {
    CANPacket p;
	int motor_serial = getIndex(motor_group, motor);
	AssemblePIDTargetSetPacket(&p, DEVICE_GROUP_MOTOR_CONTROL, motor_serial, targetPos);
	sendCANPacket(p);
}
