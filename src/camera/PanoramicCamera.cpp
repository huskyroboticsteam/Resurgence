// https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html#tutorial_homography_Demo5

#include "Camera.h"
#include "CameraParams.h"
#include "CameraConfig.h"
#include "../world_interface/motor/can_motor.h"
#include "../CAN/CAN.h"
#include "../CAN/CANMotor.h"
#include "../CAN/CANUtils.h"

#include <iostream>
#include <vector>
#include <memory>

extern "C" {
    #include <HindsightCAN/CANScience.h>
}

#define DEFAULT_CONFIG_PATH "../../camera-config/MastCameraCalibration.yml"
#define NUMBER_OF_CAPS 10
#define SERVO_NUM 0  // TODO: replace with real value

/**
 * @brief captures a frame from the panoramic camera
 * @param frame reference to cv::Mat to store frame
 * @return true if a frame was captured
 */
bool capture_frame(cv::VideoCapture& cap, cv::Mat& frame);

/**
 * @brief rotates the camera by a specified amount
 * @param angle angle in degrees to rotate to
 */
void rotate_camera(uint8_t angle);

/**
 * cli arguments:
 *      ./panoramic [camera_path] [config_path]
 */
int main(int argc, char* argv[]) {
    can::initCAN();
    
	std::string config_path = DEFAULT_CONFIG_PATH;
	if (argc == 2) {
		config_path = argv[2];
	}

	cv::VideoCapture cap;
	cam::CameraParams PARAMS;
    std::vector<std::shared_ptr<cv::Mat>> frames;

	// set up the capture size
	cv::FileStorage cam_config(config_path, cv::FileStorage::READ);

    cam_config[cam::KEY_INTRINSIC_PARAMS] >> PARAMS;
	const int w = PARAMS.getImageSize().width;
	const int h = PARAMS.getImageSize().height;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);

	if (!cam_config[cam::KEY_CAMERA_ID].empty() && !cap.open(static_cast<int>(cam_config[cam::KEY_CAMERA_ID]))) {
		std::cout << "Failed to open camera!" << std::endl;
		return EXIT_FAILURE;
	}

    for (size_t i = 0; i < NUMBER_OF_CAPS; i++) {
        std::shared_ptr<cv::Mat> frame = std::make_shared<cv::Mat>();
        if (!capture_frame(cap, *frame)) {
            std::cout << "Captured an empty frame!" << std::endl;
            // continue;
            return EXIT_FAILURE;
        }
        frames.push_back(frame);
    }



	return EXIT_SUCCESS;
}

bool capture_frame(cv::VideoCapture& cap, cv::Mat& frame) {
	cap.grab();
	cap.retrieve(frame);
	return !frame.empty();
}

void rotate_camera(uint8_t angle) {
    CANPacket p;
    uint8_t science_group = static_cast<int>(can::devicegroup_t::science);

    AssembleScienceServoPacket(&p, science_group, 0x0, SERVO_NUM,
                               angle);
    can::sendCANPacket(p);
}