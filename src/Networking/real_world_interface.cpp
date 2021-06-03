#include <future>
#include <iostream>
#include <set>

#include <opencv2/calib3d.hpp>

#include "../Globals.h"
#include "../Util.h"
#include "../ar/read_landmarks.h"
#include "../camera/Camera.h"
#include "../gps/read_usb_gps.h"
#include "../lidar/read_hokuyo_lidar.h"
#include "../log.h"
#include "../simulator/utils.h"
#include "../simulator/world_interface.h"
#include "CANUtils.h"
#include "motor_interface.h"

#include "json.hpp"

extern "C"
{
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

transform_t getOdomAt(const struct timeval &time) {
	double elapsed = getElapsedUsecs(last_odom_reading_, time) / 1000000.0;
	double delta_theta = odom_dtheta_ * elapsed;
	double rel_x, rel_y, rel_th;
	rel_th = odom_dtheta_ * elapsed;
	if (abs(odom_dtheta_) < 0.000001) {
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

constexpr double WHEEL_BASE = 1.0;				 // Distance between left and right wheels. Eyeballed
constexpr double WHEEL_RADIUS = 0.15;			 // Eyeballed
constexpr double PWM_FOR_1RAD_PER_SEC = 10000; // Eyeballed

bool setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0))
		return false;

	/* This is the inverse of the formula:
	 *		dx = (right_ground_vel + left_ground_vel) / 2
	 *		dtheta = (right_ground_vel - left_ground_vel) / WHEEL_BASE
	 */
	double right_ground_vel = dx + WHEEL_BASE / 2 * dtheta;
	double left_ground_vel = dx - WHEEL_BASE / 2 * dtheta;
	double right_angular_vel = right_ground_vel / WHEEL_RADIUS;
	double left_angular_vel = left_ground_vel / WHEEL_RADIUS;
	int16_t right_pwm = (int16_t)(right_angular_vel * PWM_FOR_1RAD_PER_SEC);
	int16_t left_pwm = (int16_t)(left_angular_vel * PWM_FOR_1RAD_PER_SEC);
	// This is a bit on the conservative side, but we heard an ominous popping sound at 20000.
	int16_t max_pwm = 15000;
	double scale_down_factor = 1.0;
	if (abs(right_pwm) > max_pwm) {
		std::cout << "WARNING: requested too-large right PWM " << right_pwm << std::endl;
		scale_down_factor = abs(right_pwm) / max_pwm;
	}
	if (abs(left_pwm) > max_pwm) {
		std::cout << "WARNING: requested too-large left PWM " << left_pwm << std::endl;
		double scale_down_factor_left = abs(left_pwm) / max_pwm;
		if (scale_down_factor_left > scale_down_factor) scale_down_factor = scale_down_factor_left;
	}
	right_pwm = (int16_t) (right_pwm / scale_down_factor);
	left_pwm = (int16_t) (left_pwm / scale_down_factor);

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
	return true;
}

points_t readLandmarks(){
	return AR::readLandmarks();
}

points_t readLidarScan(){
	if (lidar::isLidarDataFresh())
	{
		return lidar::readLidar();
	}
	else
	{
		return {};
	}
}

transform_t readOdom() {
	struct timeval now;
	gettimeofday(&now, NULL);
	return getOdomAt(now);
}

URCLeg getLeg(int index) {
	return URCLeg { 0, -1, {0.,0.,0.}};
}
