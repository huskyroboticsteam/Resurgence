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
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stitching.hpp>

// standard library imports
#include <iostream>
#include <vector>
#include <thread>

// CONSTANTS
#define COMPASS_PATH
#define ASSUMED_FOV 15  // TODO: tune this value
#define CAPTURE_ANGLE 180
#define NUMBER_OF_CAPS CAPTURE_ANGLE / ASSUMED_FOV
#define STEPPER_ID 1
#define HOME_DIRECTORY "/home/huskyrobotics/"
#define DEFAULT_CONFIG_PATH HOME_DIRECTORY "Resurgence/camera-config/MastCameraCalibration.yml"
#define SAVED_FILE_NAME HOME_DIRECTORY "panoramic.bmp"

/**
 * @brief sets up a camera in a cv::VideoCapture object
 * @param cap where the video capture device to be used for image capture
 * @param config_path the path to the camera config file
 */
bool set_up_camera(cv::VideoCapture& cap, std::string& config_path);

/**
 * @brief captures frames and puts them into a std::vector
 * @param cap the image capture device
 * @param reverse whether or not the motor should be driven in reverse
 * @param frames where the frames should be stored
 * @return true on success
 */
bool capture_frames(cv::VideoCapture& cap, bool reverse, std::vector<cv::Mat>& frames);

/**
 * @brief saves a frame to disk
 * @param frame the frame to be saved
 * @return true on success
 */
bool save_frame(const cv::Mat& frame);

/**
 * @brief saves a frame to disk
 * @param frame the frame to be saved
 * @param filename the name of the file to store the image
 * @return true on success
 */
bool save_frame(const cv::Mat& frame, const std::string& filename);

/**
 * @brief stitches together cv::Mat frames into a single panoramic image
 * @param frames reference to the vector holding the frames
 * @param stitched the final stitched panoramic photo
 * @return true on success
 */
bool stitch_frames(const std::vector<cv::Mat>& frames, cv::Mat& stitched);

/**
 * @brief adds the cardinal directions to a panoramic image
 * @param panoramic the panoramic image
 * @return true on success
 */
bool add_directions(cv::Mat& panoramic, double heading);

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
void rotate_camera(int angle);

/**
 * cli arguments:
 *      ./panoramic [camera_path] [r]
 */
int main(int argc, char* argv[]) {
    can::initCAN();
    
	// parse command line arguments
	std::string config_path = DEFAULT_CONFIG_PATH;
	if (argc < 2) {
		std::cout << "You must provide the heading of the rover." << std::endl
		<< "Usage: " << argv[0] << " heading [r/l] [image_1 image_2 ...]" << std::endl;
		return EXIT_FAILURE;
	}
	
	double heading = 0;
	try {
		heading = std::stod(argv[1]);
	} catch (const std::invalid_argument& e) {
		std::cout << "Unable to parse double for heading" << std::endl;
		return EXIT_FAILURE;
	}
	
	bool use_cam = true;
	bool reverse = false;
	std::vector<cv::Mat> frames;
	
	if (argc > 2) {
		if (argv[2][0] == 'r') {
			reverse = true;
			std::cout << "Running the camera motor in reverse!" << std::endl;
		} else if (argv[2][0]) {
			if (argc < 4) {
				std::cout << "You must specify at least one image to stitch with." << std::endl;
				return EXIT_FAILURE;
			}
			use_cam = false;
			std::cout << "Running using local images." << std::endl;
			for (int i = 3; i < argc; i++) {
				char* image_path = argv[i];
				std::cout << "Reading " << image_path << std::endl;

				cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
				if (image.empty()) {
					std::cout << "WARNING: This image could not be read, skipping." << std::endl;
					return EXIT_FAILURE;
				}
				frames.push_back(image);
			}
		}
	}

	if (use_cam) {
		// set up video capture
		cv::VideoCapture cap;

		std::cout << "Setting up camera." << std::endl;
		if (!set_up_camera(cap, config_path)) {
			return EXIT_FAILURE;
		}

		std::cout << "Taking photos." << std::endl;
		if (!capture_frames(cap, reverse, frames)) {
			return EXIT_FAILURE;
		}	
	}

	// save individual photos to disk
	for (size_t i = 0; i < (frames.size() && use_cam); i++) {
		std::stringstream filename_stream;
		filename_stream << HOME_DIRECTORY "image_" << i << ".bmp";

		std::string filename = filename_stream.str();
		std::cout << "Saving " << filename << " to disk." << std::endl;
		if (!save_frame(frames[i], filename)) {
			std::cout << "Failed to save to disk!" << std::endl;
		}
	}

	cv::Mat panoramic;
	std::cout << "Stitching images" << std::endl;
	if (!stitch_frames(frames, panoramic)) {
		std::cout << "Failed to stitch frames!" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Adding directions" << std::endl;
	if (!add_directions(panoramic, heading)) {
		std::cout << "Failed to add heading to the panoramic" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Saving to disk" << std::endl;
	if (!save_frame(panoramic)) {
		std::cout << "Failed to save panoramic image to disk!" << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

bool set_up_camera(cv::VideoCapture& cap, std::string& config_path) {
	cam::CameraParams PARAMS;
		
	// set up parameters from file
	cv::FileStorage cam_config(config_path, cv::FileStorage::READ);
	if (!cam_config.isOpened()) {
		std::cout << "Failed to open the camera config file: " << config_path << std::endl;
		return false;
	}
	cam_config[cam::KEY_INTRINSIC_PARAMS] >> PARAMS;
	int camera_id = static_cast<int>(cam_config[cam::KEY_CAMERA_ID]);
	const int w = PARAMS.getImageSize().width;
	const int h = PARAMS.getImageSize().height;

	cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);

	if (!cap.open(camera_id, cv::CAP_V4L2)) {
		std::cout << "Failed to open camera!" << std::endl;
		return false;
	}

	return true;
}

bool capture_frames(cv::VideoCapture& cap, bool reverse, std::vector<cv::Mat>& frames) {
	for (size_t i = 0; i < NUMBER_OF_CAPS; i++) {
		cv::Mat frame;
		if (!capture_frame(cap, frame)) {
			std::cout << "ERROR: Failed to capture a frame!" << std::endl;
			// continue;
			return false;
		}
		frames.push_back(frame);
		
		if (i != NUMBER_OF_CAPS - 1) {
			if (reverse) {
				rotate_camera(-ASSUMED_FOV);
			} else {
				rotate_camera(ASSUMED_FOV);
			}
		}
	}
	return true;
}

bool save_frame(const cv::Mat& frame) {
	return save_frame(frame, SAVED_FILE_NAME);
}

bool save_frame(const cv::Mat& frame, const std::string& filename) {
	return cv::imwrite(filename, frame);
}

bool stitch_frames(const std::vector<cv::Mat>& frames, cv::Mat& stitched) {
	auto stitcher = cv::Stitcher::create();  // default mode is panoramic
	cv::Stitcher::Status status = stitcher->stitch(frames, stitched);
	return status == cv::Stitcher::OK;
}

bool add_directions(cv::Mat& panoramic, double heading) {	
	cv::putText(panoramic, "N", cv::Point((heading / 360.0) * panoramic.rows + panoramic.rows / 8.0, (panoramic.cols * 5.0) / 6.0), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255, 0, 0), 5);
	cv::putText(panoramic, "E", cv::Point((heading / 360.0) * panoramic.rows + (panoramic.rows * 3.0) / 8.0, (panoramic.cols * 5.0) / 6.0), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255, 0, 0), 5);
	cv::putText(panoramic, "S", cv::Point((heading / 360.0) * panoramic.rows + (panoramic.rows * 5.0) / 8.0, (panoramic.cols * 5.0) / 6.0), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255, 0, 0), 5);
	cv::putText(panoramic, "W", cv::Point((heading / 360.0) * panoramic.rows + (panoramic.rows * 7.0) / 8.0, (panoramic.cols * 5.0) / 6.0), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1.5, CV_RGB(255, 0, 0), 5);

	return true;
}

bool capture_frame(cv::VideoCapture& cap, cv::Mat& frame) {
	cap.grab();
	cap.retrieve(frame);
	return !frame.empty();
}

void rotate_camera(int angle) {
	using namespace std::chrono_literals;
    CANPacket p;
    uint8_t science_group = static_cast<int>(can::devicegroup_t::science);
	AssembleScienceStepperTurnAnglePacket(&p, science_group, 0x3, STEPPER_ID, angle, 3);
    can::sendCANPacket(p);  // TODO: uncomment to make it move
	std::this_thread::sleep_for(50ms * ASSUMED_FOV);  // make sure it's done spinning
}
