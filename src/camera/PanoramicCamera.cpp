// https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html#tutorial_homography_Demo5
// https://docs.opencv.org/4.x/d8/d19/tutorial_stitcher.html

// the camera used is the ELP-USBFHD06H-L36
// it has a focal length of 3.6mm, which equates to an FOV of 78 degrees

// CLOOCKWISE IS WITHOUT r CLI PARAMETER

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
#include <opencv2/stitching/detail/matchers.hpp>

// standard library imports
#include <iostream>
#include <vector>
#include <thread>
#include <cmath>

// CONSTANTS
#define PANO_THRESHOLD 1.0f  // panoramic confidence threshold, 1.0 is default (.1)
#define MATCH_CONFIDENCE 0.3f  // match confidence, 0.3 is default (.5)
#define ASSUMED_FOV 30  // TODO: tune this value
#define CAMERA_FOV 78
#define NUMBER_OF_CAPS 9
#define CAPTURE_ANGLE ASSUMED_FOV * NUMBER_OF_CAPS
#define STEPPER_ID 1
#define HOME_DIRECTORY "/home/husky/"  // for the rover
// #define HOME_DIRECTORY "/home/huskyrobotics/"  // for the laptops
#define DEFAULT_CONFIG_PATH HOME_DIRECTORY "Resurgence/camera-config/MastCameraCalibration.yml"
#define SAVED_FILE_NAME HOME_DIRECTORY "panoramic.bmp"
#define DIRECTIONS_SIZE 2.0  // the size of the text for the cardinal directions

using namespace std::chrono_literals;

/**
 * @brief computes the mathematical mod of a number s.t. it is always positive
 * @param dividend the value to be divided
 * @param divisor the value to divide by
 * @return the strictly positive remainder
 */
inline int pos_mod(int dividend, int divisor);

/**
 * @brief sets up a camera in a cv::VideoCapture object
 * @param cap where the video capture device to be used for image capture
 * @param config_path the path to the camera config file
 */
bool set_up_camera(cv::VideoCapture& cap, const std::string& config_path);

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
 * @param heading the heading that the camera is facing
 * @param cam_width the width of the pictures taken by the camera
 * @return true on success
 */
bool add_directions(cv::Mat& panoramic, double heading, int cam_width);

/**
 * @brief adds a scale to corner to the top left of the image
 * @param panoramic the panoramic image
 * @param distance the distance from the landmark
 * @param cam_width the width of the pictures taked by the camera
 * @return true on success
 */
bool add_scale(cv::Mat& panoramic, int distance, int cam_width);

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
 * Usage:
 *      ./panoramic heading distance [r/l] [image_1 image_2 ...]
 */

int main(int argc, char* argv[]) {    
	// parse command line arguments
	std::string config_path = DEFAULT_CONFIG_PATH;
	if (argc < 3) {
		std::cout << "ERROR: You must provide the heading AND distance of the rover." << std::endl
		  		  << "Usage: " << std::endl
				  << "\t" << argv[0] << " heading distance [r/l] [image_1 image_2 ...]" << std::endl;
		return EXIT_FAILURE;
	}
	
	double heading;
	try {
		heading = std::stod(argv[1]);
	} catch (const std::invalid_argument& e) {
		std::cout << "ERROR: Unable to parse double for heading" << std::endl;
		return EXIT_FAILURE;
	}

	int distance;
	try {
		distance = std::stod(argv[2]);
	} catch (const std::invalid_argument& e) {
		std::cout << "ERROR: Unable to parse int for distance" << std::endl;
		return EXIT_FAILURE;
	}
	
	bool use_cam = true;
	bool reverse = false;
	std::vector<cv::Mat> frames;
	
	if (argc > 3) {
		if (argv[3][0] == 'r') {
			reverse = true;
			std::cout << "WARNING: Running the camera motor in reverse!" << std::endl;
		} else if (argv[3][0] == 'l') {
			if (argc < 4) {
				std::cout << "ERROR: You must specify at least one image to stitch with." << std::endl;
				return EXIT_FAILURE;
			}
			use_cam = false;
			std::cout << "WARNING: Running using local images." << std::endl;
			for (int i = 4; i < argc; i++) {
				char* image_path = argv[i];
				std::cout << "Reading " << image_path << std::endl;

				cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
				if (image.empty()) {
					std::cout << "WARNING: This image could not be read, skipping." << std::endl;
					return EXIT_FAILURE;
				}
				frames.push_back(image);
			}
		} else {
			std::cout << "ERROR: Unrecognized command line argument \"" << argv[3] << "\"" << std::endl;
			return EXIT_FAILURE;
		}
	}

	if (use_cam) {
		can::initCAN();

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
	for (size_t i = 0; (i < frames.size()) && use_cam; i++) {
		std::stringstream filename_stream;
		filename_stream << HOME_DIRECTORY "image_" << i << ".bmp";

		std::string filename = filename_stream.str();
		std::cout << "Saving " << filename << " to disk." << std::endl;
		if (!save_frame(frames[i], filename)) {
			std::cout << "WARNING: Failed to image to disk!" << std::endl;
		}
	}

	cv::Mat panoramic;
	std::cout << "Stitching images" << std::endl;
	if (!stitch_frames(frames, panoramic)) {
		std::cout << "ERROR: Failed to stitch frames!" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Adding directions" << std::endl;
	if (!add_directions(panoramic, heading, frames[0].cols)) {
		std::cout << "WARNING: Failed to add heading to the panoramic" << std::endl;
		// return EXIT_FAILURE;
	}

	std::cout << "Adding scale" << std::endl;
	if (!add_scale(panoramic, distance, frames[0].cols)) {
		std::cout << "WARNING: Failed to add scale bar to panoramic" << std::endl;
		// return EXIT_FAILURE;
	}

	std::cout << "Saving to disk" << std::endl;
	if (!save_frame(panoramic)) {
		std::cout << "ERROR: Failed to save panoramic image to disk!" << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

inline int pos_mod(int dividend, int divisor) {
	return (dividend % divisor + divisor) % divisor;
}

bool set_up_camera(cv::VideoCapture& cap, const std::string& config_path) {
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
	cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // should keep only one frame in the buffer, not respected
	cap.set(cv::CAP_PROP_FPS, 2);  // lower framerate, also is not being respected

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
			// std::this_thread::sleep_for(50ms * ASSUMED_FOV);  // make sure it's done spinning
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
	cv::Ptr<cv::detail::FeaturesMatcher> matcher = cv::makePtr<cv::detail::BestOf2NearestRangeMatcher>(1, true, MATCH_CONFIDENCE);
	stitcher->setFeaturesMatcher(matcher);
	stitcher->setPanoConfidenceThresh(PANO_THRESHOLD);
	cv::Stitcher::Status status = stitcher->stitch(frames, stitched);
	switch (status) {
		case cv::Stitcher::ERR_NEED_MORE_IMGS:
			std::cout << "ERROR: Stitcher thinks it needs more images" << std::endl;
			break;
		case cv::Stitcher::ERR_HOMOGRAPHY_EST_FAIL:
			std::cout << "ERROR: Homography failed" << std::endl;
			break;
		case cv::Stitcher::ERR_CAMERA_PARAMS_ADJUST_FAIL:
			std::cout << "ERROR: Camera params adjust fail" << std::endl;
			break;
		default: ;
	}
	return status == cv::Stitcher::OK;
}

bool add_directions(cv::Mat& panoramic, double heading, int cam_width) {
	const int pixels_per_degree = panoramic.cols / CAPTURE_ANGLE;
	const int starting_point = cam_width / 2;

	int north, south, east, west;  // degrees relative to heading
	north = heading;
	east = heading + 90;
	south = heading + 180;
	west = heading + 270;
	north = pos_mod(north, 360) * pixels_per_degree + starting_point;
	east = pos_mod(east, 360) * pixels_per_degree + starting_point;
	south = pos_mod(south, 360) * pixels_per_degree + starting_point;
	west = pos_mod(west, 360) * pixels_per_degree + starting_point;

	std::cout << "N: " << north << " "
			  << "E: " << east << " "
			  << "S: " << south << " "
			  << "W: " << west << " "
			  << std::endl;

	const int height = 30 * DIRECTIONS_SIZE;

	if (north > 0 && north < panoramic.cols) {
		cv::putText(panoramic, "N", cv::Point(north, height), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, DIRECTIONS_SIZE, CV_RGB(255, 0, 0), 3);
	}

	if (east > 0 && east < panoramic.cols) {
		cv::putText(panoramic, "E", cv::Point(east, height), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, DIRECTIONS_SIZE, CV_RGB(255, 0, 0), 3);
	}

	if (south > 0 && south < panoramic.cols) {
		cv::putText(panoramic, "S", cv::Point(south, height), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, DIRECTIONS_SIZE, CV_RGB(255, 0, 0), 3);
	}
	
	if (west > 0 && west < panoramic.cols) {
		cv::putText(panoramic, "W", cv::Point(west, height), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, DIRECTIONS_SIZE, CV_RGB(255, 0, 0), 3);
	}
	
	return true;
}

bool add_scale(cv::Mat& panoramic, int distance, int cam_width) {
	double meters_per_pixel = (2.0 * std::tan(CAMERA_FOV / 2.0) * distance) / cam_width;
	int scale_width = 200;  // TODO: Have the scale width equal to a power of 10
	int scale_distance = static_cast<int>(meters_per_pixel * scale_width);
	const int scale_offset = 25;
	std::cout << "MPP: " << meters_per_pixel << " scale_distance: " << scale_distance << std::endl;
	cv::line(panoramic, cv::Point(scale_offset, scale_offset), cv::Point(scale_offset + scale_width, scale_offset), CV_RGB(0, 255, 0), 2);
	cv::putText(panoramic, std::to_string(scale_distance) + " meters", cv::Point(scale_offset, scale_offset + 25), cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0, 255, 0), 2);

	return true;
}

bool capture_frame(cv::VideoCapture& cap, cv::Mat& frame) {
	for (int i = 0; i < ASSUMED_FOV * 3; i++) {
		cap.grab();
	}
	cap.retrieve(frame);
	return !frame.empty();
}

void rotate_camera(int angle) {
    CANPacket p;
    uint8_t science_group = static_cast<int>(can::devicegroup_t::science);
	AssembleScienceStepperTurnAnglePacket(&p, science_group, 0x4, STEPPER_ID, angle, 0x3);
    can::sendCANPacket(p);
}
