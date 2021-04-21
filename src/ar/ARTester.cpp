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

#include "../ThreadedCapture.h"
#include "Detector.h"

// change to true to turn on timing code
#define TIMING false

const std::string ORIG_WINDOW_NAME = "Image (Original)";

constexpr bool EXTRA_WINDOWS = true;

// Set to whichever camera params should be used
cam::CameraParams PARAMS = cam::getCameraParams(cam::Params::ROBOT_TOP_WEBCAM_480);

// Set to whichever MarkerSet should be used (CIRC/URC)
std::shared_ptr<AR::MarkerSet> MARKER_SET =
	AR::Markers::CIRC_MARKERS(); // AR::Markers::URC_MARKERS();

const std::string keys = "{h help || Show this help message.}"
						 "{params p       | 0 | The set of camera parameters to use. This "
						 "value should be a non-negative zero-indexed integer corresponding "
						 "to the member of AR::Params you want to use.}"
						 "{marker_set m   | urc | The set of markers to look for. Currently "
						 "only \"urc\" and \"circ\" are supported.}"
						 "{cam c | 0 | The camera ID to use.}"
						 "{frame_by_frame f || If present, program will go frame-by-frame "
						 "instead of capturing continuously.}";

int camera_id = 0;

std::vector<cv::Point2d> projectCube(float len, cv::Vec3d rvec, cv::Vec3d tvec)
{
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
	cv::projectPoints(object_points, rvec, tvec, PARAMS.getCameraParams(),
					  PARAMS.getDistCoeff(), image_points);

	return image_points;
}

std::vector<cv::Point2f> projectGrid(cv::Size imageSize, int spacing)
{
	cv::Point2f center(imageSize.width / 2, imageSize.height / 2);
	std::vector<cv::Point2f> grid_points;
	std::vector<cv::Point2f> projected_points;
	for (int x = 0; x < imageSize.width / 2; x += spacing)
	{
		for (int y = 0; y < imageSize.height / 2; y += spacing)
		{
			grid_points.push_back(cv::Point2f(x, y) + center);
			if (x != 0 || y != 0)
			{
				grid_points.push_back(cv::Point2f(-x, -y) + center);
				grid_points.push_back(cv::Point2f(-x, y) + center);
				grid_points.push_back(cv::Point2f(x, -y) + center);
			}
		}
	}
	cv::undistortPoints(grid_points, projected_points, PARAMS.getCameraParams(),
						PARAMS.getDistCoeff());
	return projected_points;
}

int main(int argc, char *argv[])
{
	cv::CommandLineParser parser(argc, argv, keys);
	if (!parser.check())
	{
		parser.printErrors();
	}
	parser.about("Program to open a camera and look for AR tags.");

	// print help message if "-h" or "--help" option is passed
	if (parser.has("h"))
	{
		parser.printMessage();
		return EXIT_SUCCESS;
	}

	// get camera params and marker set arguments
	int camera_param_num = parser.get<int>("p");
	std::string marker_set = parser.get<std::string>("m");
	std::transform(marker_set.begin(), marker_set.end(), marker_set.begin(), ::tolower);

	if (marker_set == "circ")
	{
		MARKER_SET = AR::Markers::CIRC_MARKERS();
	}
	else if (marker_set == "urc")
	{
		MARKER_SET = AR::Markers::URC_MARKERS();
	}
	else
	{
		std::cerr << "Unsupported marker set: \"" << marker_set << "\"" << std::endl;
		parser.printMessage();
		return EXIT_FAILURE;
	}

	cam::Params camera_to_use = cam::Params::ROBOT_TOP_WEBCAM_480;
	switch (camera_param_num)
	{
	case 0:
		camera_to_use = cam::Params::WINSTON_WEBCAM_480;
		break;
	case 1:
		camera_to_use = cam::Params::WEBCAM_1080;
		break;
	case 2:
		camera_to_use = cam::Params::WEBCAM_720;
		break;
	case 3:
		camera_to_use = cam::Params::LAPTOP;
		break;
	case 4:
		camera_to_use = cam::Params::WEBCAM;
		break;
	case 5:
		camera_to_use = cam::Params::EVAN_NEW_WEBCAM_480;
		break;
	case 6:
		camera_to_use = cam::Params::ROBOT_TOP_WEBCAM_480;
		break;
	default:
		std::cerr << "Unsupported camera params: " << std::to_string(camera_param_num)
				  << std::endl;
		parser.printMessage();
		return EXIT_FAILURE;
	}

	PARAMS = cam::getCameraParams(camera_to_use);

	camera_id = parser.get<int>("c");

	cv::Mat frame;
	size_t fnum = 0;
	AR::ThreadedCapture cap;

	int api_id = cv::CAP_ANY;

	std::cout << "Opening camera..." << std::endl;
	if (!cap.open(camera_id, api_id))
	{
		std::cerr << "ERROR! Unable to open camera" << std::endl;
		return 1;
	}

	//	cap.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	//   cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	//   cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
	cv::Size imageSize(static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
					   static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)));
	std::cout << "Image size: " << imageSize << std::endl;

	std::cout << "Opening image window, press Q to quit" << std::endl;

	cv::namedWindow(ORIG_WINDOW_NAME);

	AR::Detector detector(MARKER_SET, PARAMS);

#ifndef NDEBUG
	double loop_num = 0.0;
	long double total_time = 0.0;
	long double total_wall = 0.0;
#endif

	int count = 0;
	bool frame_by_frame = parser.has("f");

	bool show_grid = false;
	int grid_spacing = 20;

	bool loop = true;

	while (loop)
	{

#ifndef NDEBUG
		std::clock_t c_start = std::clock(); // Stores current cpu time
		auto wall_start = std::chrono::system_clock::now();
		loop_num++;
#endif

#ifndef NDEBUG
		auto read_start = std::chrono::system_clock::now();
#endif
		// Grabs frame
		if (!cap.hasNewFrame(fnum))
		{
			continue;
		}
		cap.read(frame);
#ifndef NDEBUG
		auto read_end = std::chrono::system_clock::now();
#endif
		if (frame.empty())
		{
			std::cerr << "ERROR! Blank frame grabbed" << std::endl;
			continue;
		}
#ifndef NDEBUG
		if (TIMING)
		{
			long double read_time =
				std::chrono::duration_cast<std::chrono::milliseconds>(read_end - read_start)
					.count();
			std::cout << "Read took " << read_time << "ms" << std::endl;
		}
#endif

		// Mats to hold grayscale and edge detection results
		cv::Mat gray;
		cv::Mat edges;

		std::vector<std::vector<cv::Point2f>> quad_corners;

		// Passes frame to the detector class.
		// Tags will be located and returned.
		std::vector<AR::Tag> tags = detector.detectTags(frame);

		// Draws a green line around all quadrilateral shapes found in the image (debugging
		// purposes)
		cv::Scalar green_line = cv::Scalar(0, 255, 0);
		for (std::vector<cv::Point2f> quad : quad_corners)
		{
			cv::line(frame, quad[0], quad[1], green_line, 2);
			cv::line(frame, quad[1], quad[2], green_line, 2);
			cv::line(frame, quad[2], quad[3], green_line, 2);
			cv::line(frame, quad[3], quad[0], green_line, 2);
		}

		// Draws an outline around the tag and a cross in the center
		// Projects a cube onto the tag to debug TVec and RVec
		for (AR::Tag tag : tags)
		{
			std::cout << "Tag ID: " << tag.getMarker().getId() << std::endl;
			std::vector<cv::Point2d> cubePoints =
				projectCube(MARKER_SET->getPhysicalSize(), tag.getRVec(), tag.getTVec());
			std::cout << "rvec: " << tag.getRVec() << std::endl;
			cv::Vec3d tvec = tag.getTVec();
			double dist = sqrt(pow(tvec[0], 2) + pow(tvec[1], 2) + pow(tvec[2], 2));
			std::cout << "tvec: " << tvec << "(distance: " << dist << ")" << std::endl;
			std::cout << "coordinates: " << tag.getCoordinates() << std::endl;

			for (size_t i = 0; i < 4; i++)
			{
				size_t next = (i == 3 ? 0 : i + 1);
				cv::line(frame, cubePoints[i], cubePoints[next], cv::Scalar(0, 0, 255), 3);
				cv::line(frame, cubePoints[i], cubePoints[i + 4], cv::Scalar(0, 255, 0), 3);
				cv::line(frame, cubePoints[i + 4], cubePoints[next + 4], cv::Scalar(255, 0, 0),
						 3);
			}
		}

		if (show_grid)
		{
			std::vector<cv::Point2f> grid = projectGrid(imageSize, grid_spacing);
			for (cv::Point2f pt : grid)
			{
				cv::Point2f newPt(pt.x * imageSize.width, pt.y * imageSize.height);
				cv::drawMarker(frame, newPt, cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 10, 1);
			}
		}

		cv::imshow(ORIG_WINDOW_NAME, frame);

		int delay = (frame_by_frame && count % 10 == 0) ? 0 : 1;
		switch (cv::waitKey(delay))
		{
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
		default:
			break;
		}

#ifndef NDEBUG
		if (TIMING)
		{
			// Calculates time used to complete one loop on average
			std::clock_t c_end = std::clock();
			auto wall_end = std::chrono::system_clock::now();
			auto wall_t =
				std::chrono::duration_cast<std::chrono::microseconds>(wall_end - wall_start)
					.count() /
				1000.0;
			long double cpu_t = 1000.0 * (c_end - c_start) / CLOCKS_PER_SEC;
			total_time += cpu_t;
			total_wall += wall_t;
			std::cout << "CPU Time: " << cpu_t << "ms, avg: " << total_time / loop_num
					  << "ms; Wall Time: " << wall_t << "ms, avg: " << total_wall / loop_num
					  << "ms" << std::endl;
		}
#endif
	}

	return 0;
}
