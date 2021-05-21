#include <iostream>

#include "../Globals.h"
#include "../camera/Camera.h"
#include "../ar/Detector.h"
#include "../log.h"
#include "../simulator/utils.h"
#include "../simulator/world_interface.h"
#include "CANUtils.h"
#include "motor_interface.h"
#include <set>

#include "json.hpp"

extern "C" {
#include "../HindsightCAN/CANMotorUnit.h"
}

using nlohmann::json;

cam::Camera ar_cam;
AR::Detector ar_detector(AR::Markers::URC_MARKERS(), cam::CameraParams());

void world_interface_init() {
	try {
		ar_cam.openFromConfigFile(Constants::AR_CAMERA_CONFIG_PATH);
		if(!ar_cam.hasIntrinsicParams()){
			log(LOG_ERROR, "Camera must have intrinsic parameters for AR tag detection\n");
			ar_cam = cam::Camera();
		} else {
			ar_detector =
				AR::Detector(AR::Markers::URC_MARKERS(), ar_cam.getIntrinsicParams());
		}
	} catch (std::exception e) {
		log(LOG_ERROR, "Error opening camera for AR tag detection:\n%s\n", e.what());
	}
}

constexpr double WHEEL_BASE = 1.0;			   // Distance between left and right wheels. Eyeballed
constexpr double WHEEL_RADIUS = 0.15;		   // Eyeballed
constexpr double PWM_FOR_1RAD_PER_SEC = 10000; // Eyeballed

bool setCmdVel(double dtheta, double dx) {
	if (Globals::E_STOP && (dtheta != 0 || dx != 0))
		return false;

	/* This is the inverse of the formula:
	 *    dx = (right_ground_vel + left_ground_vel) / 2
	 *    dtheta = (right_ground_vel - left_ground_vel) / WHEEL_BASE
	 */
	double right_ground_vel = dx + WHEEL_BASE / 2 * dtheta;
	double left_ground_vel = dx - WHEEL_BASE / 2 * dtheta;
	double right_angular_vel = right_ground_vel / WHEEL_RADIUS;
	double left_angular_vel = left_ground_vel / WHEEL_RADIUS;
	int16_t right_pwm = (int16_t)(right_angular_vel * PWM_FOR_1RAD_PER_SEC);
	int16_t left_pwm = (int16_t)(left_angular_vel * PWM_FOR_1RAD_PER_SEC);
	// This is a bit on the conservative side, but we heard an ominous popping sound at 20000.
	int16_t max_pwm = 15000;
	if (abs(right_pwm) > max_pwm) {
		std::cout << "WARNING: requested too-large right PWM " << right_pwm << std::endl;
		right_pwm = max_pwm * (right_pwm < 0 ? -1 : 1);
	}
	if (abs(left_pwm) > max_pwm) {
		std::cout << "WARNING: requested too-large left PWM " << left_pwm << std::endl;
		left_pwm = max_pwm * (left_pwm < 0 ? -1 : 1);
	}

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

// Everything below this line still needs to be implemented
points_t last_landmarks;
uint32_t last_frame_no = 0;
constexpr size_t NUM_LANDMARKS = 11;
points_t readLandmarks() {
	if (ar_cam.isOpen()) {
		if(ar_cam.hasNext(last_frame_no)){
			cv::Mat frame;
			ar_cam.next(frame, last_frame_no);
			std::vector<AR::Tag> tags = ar_detector.detectTags(frame);
		    point_t output[NUM_LANDMARKS];
			std::map<int, AR::Tag> first_tag;
			for(AR::Tag tag : tags){
				int id = tag.getMarker().getId();
				if(first_tag.find(id) != first_tag.end()){
					first_tag[id] = tag;
				}
			}
			for(int i = 0; i < NUM_LANDMARKS; i++){
				if(first_tag.find(i) != first_tag.end()){
					cv::Vec3d coords = first_tag[i].getCoordinates();
					output[i] = {coords[2], -coords[0], 1.0};
				}
			}
		}
		return last_landmarks;
	} else {
		return {};
	}
}

transform_t readGPS() {
	return toTransform({0, 0, 0});
}

transform_t readOdom() {
	return toTransform({0, 0, 0});
}

URCLeg getLeg(int index) {
  return URCLeg { 0, -1, {0.,0.,0.}};
}
