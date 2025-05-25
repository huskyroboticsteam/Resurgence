// https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html#tutorial_homography_Demo5
// https://docs.opencv.org/4.x/d8/d19/tutorial_stitcher.html

// husky robotics imports
#include "Camera.h"
#include "CameraParams.h"
#include "CameraConfig.h"
#include "../world_interface/motor/can_motor.h"
#include "../CAN/CAN.h"
#include "../CAN/CANMotor.h"
#include "../CAN/CANUtils.h"

extern "C" {
    #include <HindsightCAN/CANScience.h>
}

// opencv imports
#include "../opencv4/opencv2/imgcodecs.hpp"
#include "../opencv4/opencv2/stitching.hpp"

// standard library imports
#include <iostream>
#include <vector>

// CONSTANTS
#define DEFAULT_CONFIG_PATH "~/Resurgence/camera-config/MastCameraCalibration.yml"
#define ASSUMED_FOV 60  // TODO: replace with real FOV of camera
#define NUMBER_OF_CAPS 360 / ASSUMED_FOV
#define STEPPER_ID 0  // TODO: replace with real value
#define SAVED_FILE_NAME "~/panoramic.jpg"

/**
 * @brief saves a frame to disk
 * @param frame the frame to be saved
 * @return true on success
 */
bool save_frame(const cv::Mat& frame);

/**
 * @brief stitches together cv::Mat frames into a single panoramic image
 * @param frames reference to the vector holding the frames
 * @param stitched the final stitched panoramic photo
 * @return true on success
 */
bool stitch_frames(const std::vector<cv::Mat>& frames, cv::Mat& stitched);

/**
 * @brief captures a frame from the panoramic camera
 * @param cap the device to capture from
 * @param frame output reference to cv::Mat to store frame
 * @return true on success
 */
bool capture_frame(cv::VideoCapture& cap, cv::Mat& frame);

/**
 * @brief rotates the camera to a specific position
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
    std::vector<cv::Mat> frames;

	// set up parameters from file
	// TODO: Figure out why the config file is not opening
	// cv::FileStorage cam_config(config_path, cv::FileStorage::READ);
	// if (!cam_config.isOpened()) {
	// 	std::cout << "Failed to open the camera config file: " << config_path << std::endl;
	// 	return EXIT_FAILURE;
	// }
    // cam_config[cam::KEY_INTRINSIC_PARAMS] >> PARAMS;
	// int camera_id = static_cast<int>(cam_config[cam::KEY_CAMERA_ID]);
	// const int w = PARAMS.getImageSize().width;
	// const int h = PARAMS.getImageSize().height;

	const int camera_id = 40;
	const int w = 640;
	const int h = 480;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);

	if (!cap.open(camera_id)) {
		std::cout << "Failed to open camera!" << std::endl;
		return EXIT_FAILURE;
	}

	// std::stringstream gst;
	// gst << "v4l2src device=/dev/video40 ! image/jpeg,width=640,height=480,framerate=30/1 ! jpegdec ! videoconvert ! appsink";
	// cap.open(40, gst, )

	rotate_camera(0);
    for (size_t i = 0; i < NUMBER_OF_CAPS; i++) {
        cv::Mat frame;
        if (!capture_frame(cap, frame)) {
            std::cout << "Failed to capture a frame!" << std::endl;
            // continue;
            return EXIT_FAILURE;
        }
        frames.push_back(frame);
		rotate_camera(i * ASSUMED_FOV);
    }

	cv::Mat stitched;
	if (!stitch_frames(frames, stitched)) {
		std::cout << "Failed to stitch frames!" << std::endl;
		return EXIT_FAILURE;
	}

	if (!save_frame(stitched)) {
		std::cout << "Failed to save panoramic image to disk!" << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

bool save_frame(const cv::Mat& frame) {
	return cv::imwrite(SAVED_FILE_NAME, frame);
}

bool stitch_frames(const std::vector<cv::Mat>& frames, cv::Mat& stitched) {
	auto stitcher = cv::Stitcher::create();  // default mode is panoramic
	cv::Stitcher::Status status = stitcher->stitch(frames, stitched);
	if (status != cv::Stitcher::OK) return false;
	return true;
}

bool capture_frame(cv::VideoCapture& cap, cv::Mat& frame) {
	cap.grab();
	cap.retrieve(frame);
	return !frame.empty();
}

void rotate_camera(uint8_t angle) {
    CANPacket p;
    uint8_t science_group = static_cast<int>(can::devicegroup_t::science);

	AssembleScienceStepperTurnAnglePacket(&p, science_group, 0x0, STEPPER_ID, angle, 3);
    can::sendCANPacket(p);
}
