#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
#include "../camera/CameraParams.h"
#include "Detector.h"

#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

constexpr int NUM_SKIP = 5;

const std::string WINDOW_NAME = "Image";

const std::string keys =
	"{h help             |        | Show this help message.}"
	"{cam_config c       | <none> | The path to a camera configuration file "
	"defining the camera you would like to use.}"
	"{file_override fo   |        | If present, should be the name of a video file to open "
	"rather than the one defined in the configuration file.}"
	"{cam_override co    | -1     | If present, should be the ID of a camera to open "
	"rather than the one defined in the configuration file. NOTE: Will take precedence over "
	"file_override if both are present.}"
	"{marker_set m       | urc    | The set of markers to look for. Currently "
	"only \"urc\" and \"circ\" are supported.}"
	"{frame_by_frame f   |        | If present, program will go frame-by-frame "
	"instead of capturing continuously.}";

// TODO: come up with a better solution for examining video files
// cam::Camera cap;
cv::VideoCapture cap;
cam::CameraParams PARAMS;
std::shared_ptr<AR::MarkerSet> MARKER_SET;

std::vector<cv::Point2d> projectCube(float len, cv::Vec3d rvec, cv::Vec3d tvec);
std::vector<cv::Point2f> projectGrid(cv::Size imageSize, int spacing);
static inline void noValue(std::string option);

int main(int argc, char* argv[]) {
	cv::CommandLineParser parser(argc, argv, keys);
	if (!parser.check()) {
		parser.printErrors();
	}
	parser.about("Program to open a camera and look for AR tags.");

	// print help message if "-h" or "--help" option is passed
	if (parser.has("h")) {
		parser.printMessage();
		return EXIT_SUCCESS;
	}

	if (!parser.has("c") || parser.get<std::string>("c").empty()) {
		std::cerr << "Error: camera configuration file is required." << std::endl;
		parser.printMessage();
		return EXIT_FAILURE;
	}

	std::string marker_set = parser.get<std::string>("m");
	std::transform(marker_set.begin(), marker_set.end(), marker_set.begin(), ::tolower);

	if (marker_set == "circ") {
		MARKER_SET = AR::Markers::CIRC_MARKERS();
	} else if (marker_set == "urc") {
		MARKER_SET = AR::Markers::URC_MARKERS();
	} else if (marker_set.empty()) {
		noValue("marker_set");
	} else {
		std::cerr << "Unsupported marker set: \"" << marker_set << "\"" << std::endl;
		parser.printMessage();
		return EXIT_FAILURE;
	}

	int cam_id = parser.get<int>("co", -1);
	std::string cam_file = parser.get<std::string>("fo");
	cv::FileStorage cam_config(parser.get<std::string>("c"), cv::FileStorage::READ);
	if (!cam_config.isOpened()) {
		std::cerr << "Error: given camera configuration file " << parser.get<std::string>("c")
				  << " does not exist!" << std::endl;
		cam_config.release();
		return EXIT_FAILURE;
	}
	if (cam_config[cam::KEY_INTRINSIC_PARAMS].empty()) {
		std::cerr << "Error: Intrinsic parameters are required for AR tag detection!"
				  << std::endl;
		cam_config.release();
		return EXIT_FAILURE;
	}
	cam_config[cam::KEY_INTRINSIC_PARAMS] >> PARAMS;
	// read filename or camera ID, and open camera.
	if (!cam_config[cam::KEY_FILENAME].empty() && !parser.has("fo")) {
		cam_file = (std::string)cam_config[cam::KEY_FILENAME];
	} else if (!cam_config[cam::KEY_CAMERA_ID].empty() && !parser.has("co")) {
		cam_id = static_cast<int>(cam_config[cam::KEY_CAMERA_ID]);
	} else if (!parser.has("fo") && !parser.has("co")) {
		std::cerr << "Error: no file or camera_id was provided in the configuration file or "
					 "on the command line!"
				  << std::endl;
		std::cerr << "Usage:" << std::endl;
		parser.printMessage();
		return EXIT_FAILURE;
	}
	cam_config.release();

	cv::Mat frame;
	[[maybe_unused]] uint32_t fnum = 0;

	std::cout << "Opening camera..." << std::endl;
	bool open_success = false;
	if (cam_id > -1) {
		open_success = cap.open(cam_id);
	} else if (!cam_file.empty()) {
		open_success = cap.open(cam_file);
	}

	if (!open_success) {
		std::cerr << "ERROR! Unable to open camera" << std::endl;
		return EXIT_FAILURE;
	}

	const int w = PARAMS.getImageSize().width;
	const int h = PARAMS.getImageSize().height;
	cap.set(cv::CAP_PROP_FRAME_WIDTH, w);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, h);
	std::cout << "Set image dimensions to " << w << " x " << h << std::endl;

	// [Added]: Print keyboard controls
	std::cout << "\n=== Keyboard Controls ===" << std::endl;
	std::cout << "  Q - Quit" << std::endl;
	std::cout << "  A - Toggle AR detection on/off" << std::endl;
	std::cout << "  G - Grid on" << std::endl;
	std::cout << "  H - Grid off" << std::endl;
	std::cout << "  R - Show rejected points" << std::endl;
	std::cout << "  L - Hide rejected points" << std::endl;
	std::cout << "========================\n" << std::endl;

	std::cout << "Opening image window, press Q to quit" << std::endl;

	cv::namedWindow(WINDOW_NAME);

	AR::Detector detector(MARKER_SET, PARAMS);

	int count = 0;
	bool frame_by_frame = parser.has("f");

	bool show_grid = false;
	int grid_spacing = 20;
	bool show_rejected = false;
	// [Added]: Toggle for enabling/disabling AR detection at runtime
	bool ar_detection_enabled = true;

	bool loop = true;
	cv::Size imageSize = PARAMS.getImageSize();

	while (loop) {
		// Grabs frame
		if (!cap.grab()) {
			continue;
		}
		cap.retrieve(frame);
		if (frame.empty()) {
			std::cerr << "ERROR! Blank frame grabbed" << std::endl;
			continue;
		}

		// Passes frame to the detector class.
		// Tags will be located and returned.
		// [Changed]: Only detect tags if AR detection is enabled
		std::vector<std::vector<cv::Point2f>> rejected;
		std::vector<AR::Tag> tags;
		if (ar_detection_enabled) {
			tags = detector.detectTags(frame, rejected, false);
		}

		// Draws an outline around the tag and a cross in the center
		// Projects a cube onto the tag to debug TVec and RVec
		for (AR::Tag tag : tags) {
			std::cout << "Tag ID: " << tag.getMarker().getId() << std::endl;
			std::vector<cv::Point2d> cubePoints =
				projectCube(MARKER_SET->getPhysicalSize(), tag.getRVec(), tag.getTVec());
			std::cout << "rvec: " << tag.getRVec() << std::endl;
			cv::Vec3d tvec = tag.getTVec();
			double dist = sqrt(pow(tvec[0], 2) + pow(tvec[1], 2) + pow(tvec[2], 2));
			std::cout << "tvec: " << tvec << "(distance: " << dist << ")" << std::endl;
			std::cout << "coordinates: " << tag.getCoordinates() << std::endl;

			for (size_t i = 0; i < 4; i++) {
				size_t next = (i == 3 ? 0 : i + 1);
				cv::line(frame, cubePoints[i], cubePoints[next], cv::Scalar(0, 0, 255), 3);
				cv::line(frame, cubePoints[i], cubePoints[i + 4], cv::Scalar(0, 255, 0), 3);
				cv::line(frame, cubePoints[i + 4], cubePoints[next + 4], cv::Scalar(255, 0, 0),
						 3);
			}
		}

		if (show_rejected) {
			for (const auto& points : rejected) {
				for (size_t i = 0; i < points.size() - 1; i++) {
					const auto& p1 = points[i];
					const auto& p2 = points[i + 1];
					cv::line(frame, p1, p2, cv::Scalar(255, 0, 255));
				}
			}
		}

		if (show_grid) {
			std::vector<cv::Point2f> grid = projectGrid(imageSize, grid_spacing);
			for (cv::Point2f pt : grid) {
				cv::Point2f newPt(pt.x * static_cast<float>(imageSize.width),
								  pt.y * static_cast<float>(imageSize.height));
				cv::drawMarker(frame, newPt, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 10, 1);
			}
		}

		cv::imshow(WINDOW_NAME, frame);

		int delay = (frame_by_frame && count % NUM_SKIP == 0) ? 0 : 1;
		switch (cv::waitKey(delay)) {
			case 'q':
				loop = false;
				break;
			case 'g':
				show_grid = true;
				std::cout << "Grid on" << std::endl;
				break;
			case 'h':
				show_grid = false;
				std::cout << "Grid off" << std::endl;
				break;
			case 'r':
				show_rejected = true;
				std::cout << "Rejected points on" << std::endl;
				break;
			case 'l':
				show_rejected = false;
				std::cout << "Rejected points off" << std::endl;
				break;
			// [Added]: Toggle AR detection on/off
			case 'a':
				ar_detection_enabled = !ar_detection_enabled;
				std::cout << "AR detection " << (ar_detection_enabled ? "ON" : "OFF") << std::endl;
				break;
			default:
				break;
		}
		count++;
	}

	return 0;
}

std::vector<cv::Point2d> projectCube(float len, cv::Vec3d rvec, cv::Vec3d tvec) {
	std::vector<cv::Point3d> object_points;
	std::vector<cv::Point2d> image_points;

	object_points.push_back(cv::Point3d(-(len / 2), (len / 2), 0));
	object_points.push_back(cv::Point3d((len / 2), (len / 2), 0));
	object_points.push_back(cv::Point3d((len / 2), -(len / 2), 0));
	object_points.push_back(cv::Point3d(-(len / 2), -(len / 2), 0));
	object_points.push_back(cv::Point3d(-(len / 2), (len / 2), len));
	object_points.push_back(cv::Point3d((len / 2), (len / 2), len));
	object_points.push_back(cv::Point3d((len / 2), -(len / 2), len));
	object_points.push_back(cv::Point3d(-(len / 2), -(len / 2), len));
	cv::projectPoints(object_points, rvec, tvec, PARAMS.getCameraMatrix(),
					  PARAMS.getDistCoeff(), image_points);

	return image_points;
}

std::vector<cv::Point2f> projectGrid(cv::Size imageSize, int spacing) {
	cv::Point2f center(static_cast<float>(imageSize.width) / 2.0f,
					   static_cast<float>(imageSize.height) / 2.0f);
	std::vector<cv::Point2f> grid_points;
	std::vector<cv::Point2f> projected_points;
	float xf, yf;
	for (int x = 0; x < imageSize.width / 2; x += spacing) {
		for (int y = 0; y < imageSize.height / 2; y += spacing) {
			xf = static_cast<float>(x);
			yf = static_cast<float>(y);
			grid_points.push_back(cv::Point2f(xf, yf) + center);
			if (x != 0 || y != 0) {
				grid_points.push_back(cv::Point2f(-xf, -yf) + center);
				grid_points.push_back(cv::Point2f(-xf, yf) + center);
				grid_points.push_back(cv::Point2f(xf, -yf) + center);
			}
		}
	}
	cv::undistortPoints(grid_points, projected_points, PARAMS.getCameraMatrix(),
						PARAMS.getDistCoeff());
	return projected_points;
}

static inline void noValue(std::string option) {
	std::cerr << "Error: No value given for " << option << " option!" << std::endl
			  << "Please remember to use '=' between flags and values." << std::endl;
	exit(EXIT_FAILURE);
}
