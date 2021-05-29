#include <iostream>

#include "../Globals.h"
#include "../camera/Camera.h"
#include "../ar/Detector.h"
#include "../log.h"
#include "../simulator/utils.h"
#include "../simulator/world_interface.h"
#include "../gps/read_usb_gps.h"
#include "json.hpp"
#include "motor_interface.h"
#include "CANUtils.h"
#include "motor_interface.h"
#include <set>
#include <opencv2/calib3d.hpp>
#include "../Util.h"

#include "json.hpp"

extern "C" {
#include "../HindsightCAN/CANMotorUnit.h"
}

using nlohmann::json;

cam::Camera ar_cam;
AR::Detector ar_detector(AR::Markers::URC_MARKERS(), cam::CameraParams());
points_t zero_landmarks;
const point_t ZERO_POINT = {0.0, 0.0, 0.0};
constexpr size_t NUM_LANDMARKS = 11;

void world_interface_init() {
	for(int i = 0; i < NUM_LANDMARKS; i++){
		zero_landmarks.push_back({0.0, 0.0, 0.0});
	}
	try {
		ar_cam.openFromConfigFile(Constants::AR_CAMERA_CONFIG_PATH);
		if(!ar_cam.hasIntrinsicParams()){
			log(LOG_ERROR, "Camera does not have intrinsic parameters! AR tag detection "
						   "cannot be performed.\n");
		} else {
			ar_detector =
				AR::Detector(AR::Markers::URC_MARKERS(), ar_cam.getIntrinsicParams());
		}
		if(!ar_cam.hasExtrinsicParams()){
			log(LOG_WARN, "Camera does not have extrinsic parameters! Coordinates returned "
						  "for AR tags will be relative to camera\n");
		}
	} catch (std::exception& e) {
		log(LOG_ERROR, "Error opening camera for AR tag detection:\n%s\n", e.what());
	}
	bool gps_success = startGPSThread();
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
points_t last_landmarks = zero_landmarks;
uint32_t last_frame_no = 0;
cv::Mat frame;
points_t readLandmarks() {
	// uncomment for timing of readLandmarks
	// util::ScopedTimer("readLandmarks");

	// camera might not be open if opening camera failed in init_world_interface.
	// if it is not, return a list of 11 zero points.
	if (ar_cam.isOpen()) {
		// if we have a next frame, retrieve it and perform AR tag detection.
		if (ar_cam.hasNext(last_frame_no)) {
			ar_cam.next(frame, last_frame_no);
			std::vector<AR::Tag> tags = ar_detector.detectTags(frame);
			log(LOG_DEBUG, "readLandmarks(): %d tags spotted\n", tags.size());

			// build up map with first tag of each ID spotted.
			points_t output(NUM_LANDMARKS);
			std::map<int, cv::Vec3d> ids_to_camera_coords;
			for (AR::Tag tag : tags) {
				int id = tag.getMarker().getId();
				if (ids_to_camera_coords.find(id) == ids_to_camera_coords.end()) {
					ids_to_camera_coords[id] = tag.getCoordinates();
				}
			}

			// for every possible landmark ID, go through and if the map contains a value for
			// that ID, add it to the output array (doing appropriate coordinate space
			// transforms). If not, add a zero point.
			for (int i = 0; i < NUM_LANDMARKS; i++) {
				if (ids_to_camera_coords.find(i) != ids_to_camera_coords.end()) {
					cv::Vec3d coords = ids_to_camera_coords[i];
					// if we have extrinsic parameters, multiply coordinates by them to do
					// appropriate transformation.
					if (ar_cam.hasExtrinsicParams()) {
						cv::Vec4d coords_homogeneous = {coords[0], coords[1], coords[2], 1};
						cv::Mat transformed =
							ar_cam.getExtrinsicParams() * cv::Mat(coords_homogeneous);
						output[i] = {transformed.at<double>(0, 0),
									 transformed.at<double>(1, 0), 1.0};
					} else {
						// just account for coordinate axis change; rover frame has +x front,
						// +y left, +z up while camera has +x right, +y down, +z front.
						output[i] = {coords[2], -coords[0], 1.0};
					}
				} else {
					output[i] = ZERO_POINT;
				}
			}

			// return output array.
			last_landmarks = output;
			return output;
		} else {
			return last_landmarks;
		}
	} else {
		return zero_landmarks;
	}
}

transform_t readOdom() {
	return toTransform({0, 0, 0});
}

URCLeg getLeg(int index) {
  return URCLeg { 0, -1, {0.,0.,0.}};
}
