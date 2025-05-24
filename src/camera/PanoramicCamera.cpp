// https://docs.opencv.org/4.x/d9/dab/tutorial_homography.html#tutorial_homography_Demo5

#include "Camera.h"
#include "CameraParams.h"

#include <iostream>
#include <vector>
#include <memory>

#define DEFAULT_CAMERA_PATH "PATH"
#define DEFAULT_CONFIG_PATH "PATH"
#define NUMBER_OF_CAPS 10
/**
 * @brief captures a frame from the panoramic camera
 * @param frame reference to cv::Mat to store frame
 * @return true if a frame was captured
 */
bool capture_frame(cv::VideoCapture& cap, cv::Mat& frame);

/**
 * cli arguments:
 *      ./panoramic [camera_path] [config_path]
 */
int main(int argc, char* argv[]) {
	std::string camera_path = DEFAULT_CAMERA_PATH;
	std::string config_path = DEFAULT_CONFIG_PATH;
	if (argc == 3) {
		camera_path = argv[1];
		config_path = argv[2];
	}

	cv::VideoCapture cap;
	cam::CameraParams PARAMS;
    std::vector<std::shared_ptr<cv::Mat>> frames;

	// set up the capture size
	cv::FileStorage cam_config(config_path, cv::FileStorage::READ);
	const int w = PARAMS.getImageSize().width;
	const int h = PARAMS.getImageSize().height;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);

	if (!cap.open(camera_path)) {
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
