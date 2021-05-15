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

#include "../camera/Camera.h"
#include "../camera/CameraParams.h"
#include "Detector.h"

const std::string WINDOW_NAME = "Image";

const std::string keys =
	"{h help             |        | Show this help message.}"
	"{cam_config c       | <none> | The path to a camera configuration file "
	"defining the camera you would like to use.}"
	"{cam_override co    | -1     | If present, should be the ID of a camera to open "
	"rather than the one defined in the configuration file.}"
	"{marker_set m       | urc    | The set of markers to look for. Currently "
	"only \"urc\" and \"circ\" are supported.}"
	"{frame_by_frame f   |        | If present, program will go frame-by-frame "
	"instead of capturing continuously.}";

cam::Camera cap;
cam::CameraParams PARAMS;
std::shared_ptr<AR::MarkerSet> MARKER_SET;

std::vector<cv::Point2d> projectCube(float len, cv::Vec3d rvec, cv::Vec3d tvec);
std::vector<cv::Point2f> projectGrid(cv::Size imageSize, int spacing);

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

	cv::FileStorage cam_config(parser.get<std::string>("c"), cv::FileStorage::READ);
	if (cam_config[cam::KEY_INTRINSIC_PARAMS].empty())
	{
		std::cerr << "Intrinsic parameters are required for AR tag detection!" << std::endl;
		cam_config.release();
		return EXIT_FAILURE;
	}
	cam_config[cam::KEY_INTRINSIC_PARAMS] >> PARAMS;
	cam_config.release();

	cv::Mat frame;
	uint32_t fnum = 0;

	std::cout << "Opening camera..." << std::endl;
	int cam_override_id = parser.get<int>("co");
	bool open_success = false;
	if (cam_override_id > -1)
	{
		open_success = cap.open(cam_override_id, PARAMS);
	}
	else
	{
		try
		{
			cap = cam::Camera::openFromConfigFile(parser.get<std::string>("c"));
		}
		catch (cam::invalid_camera_config c)
		{
			std::cerr << c.what() << std::endl;
		}
	}

	if (!open_success)
	{
		std::cerr << "ERROR! Unable to open camera" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Opening image window, press Q to quit" << std::endl;

	cv::namedWindow(WINDOW_NAME);

	AR::Detector detector(MARKER_SET, PARAMS);

	int count = 0;
	bool frame_by_frame = parser.has("f");

	bool show_grid = false;
	int grid_spacing = 20;

	bool loop = true;
	cv::Size imageSize = PARAMS.getImageSize();

	while (loop)
	{
		// Grabs frame
		if (!cap.hasNext(fnum))
		{
			continue;
		}
		cap.next(frame, fnum);
		if (frame.empty())
		{
			std::cerr << "ERROR! Blank frame grabbed" << std::endl;
			continue;
		}

		// Passes frame to the detector class.
		// Tags will be located and returned.
		std::vector<AR::Tag> tags = detector.detectTags(frame);

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

		cv::imshow(WINDOW_NAME, frame);

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
	}

	return 0;
}

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
	cv::projectPoints(object_points, rvec, tvec, PARAMS.getCameraMatrix(),
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
	cv::undistortPoints(grid_points, projected_points, PARAMS.getCameraMatrix(),
						PARAMS.getDistCoeff());
	return projected_points;
}
