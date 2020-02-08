#include "Detector.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <ctime>

const std::string ORIG_WINDOW_NAME = "Image (Original)";
const std::string THRESH_TRACKBAR_NAME = "Threshold";
const std::string THRESH2_TRACKBAR_NAME = "Threshold 2";
const std::string BLUR_TRACKBAR_NAME = "Blur";

constexpr bool EXTRA_WINDOWS = true;

// Set to whichever camera params should be used
const AR::CameraParams PARAMS = AR::WEBCAM_PARAMS;

int camera_id = 0;

std::vector<cv::Point2d> projectCube(double len, cv::Vec3d rvec, cv::Vec3d tvec)
{
	std::vector<cv::Point3d> object_points;
	std::vector<cv::Point2d> image_points;
	
	object_points.push_back(cv::Point3d(-(len/2), (len/2), 0));
	object_points.push_back(cv::Point3d((len/2), (len/2), 0));
	object_points.push_back(cv::Point3d((len/2), -(len/2), 0));
	object_points.push_back(cv::Point3d(-(len/2), -(len/2), 0));
	object_points.push_back(cv::Point3d(-(len/2), (len/2), len));
	object_points.push_back(cv::Point3d((len/2), (len/2), len));
	object_points.push_back(cv::Point3d((len/2), -(len/2), len));
	object_points.push_back(cv::Point3d(-(len/2), -(len/2), len));
	cv::projectPoints(object_points, rvec, tvec, PARAMS.getCameraParams(),
					  PARAMS.getDistCoeff(), image_points);
	
	return image_points;
}

int main(int argc, char *argv[])
{
	if (argc > 1)
	{
		camera_id = std::stoi(argv[1]);
	}

	cv::Mat frame;
	cv::VideoCapture cap;

	int api_id = cv::CAP_ANY;

	int thresh_val = 50;
	int thresh2_val = 120;
	int blur_val = 2;

	std::cout << "Opening camera..." << std::endl;
	
	cap.open(camera_id + api_id);
	if (!cap.isOpened())
	{
		std::cerr << "ERROR! Unable to open camera" << std::endl;
		return 1;
	}

	//   cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	//   cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	std::cout << "Opening image window, press Q to quit" << std::endl;

	cv::namedWindow(ORIG_WINDOW_NAME);
	cv::createTrackbar(THRESH_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh_val, 256);
	cv::createTrackbar(THRESH2_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh2_val, 256);
	cv::createTrackbar(BLUR_TRACKBAR_NAME, ORIG_WINDOW_NAME, &blur_val, 10);

	AR::Detector detector(PARAMS);

	double loop_num = 0.0;
	long double total_time = 0.0;
	long double time = 0.0;

	while (true)
	{	
		std::clock_t c_start = std::clock(); // Stores current cpu time
		loop_num++;

		cap.read(frame);
		if (frame.empty())
		{
			std::cerr << "ERROR! Blank frame grabbed" << std::endl;
			break;
		}

		// Mats to hold grayscale and edge detection results
		cv::Mat gray;
		cv::Mat edges;

		// pass frame to detector here
		std::vector<AR::Tag> tags =
		    detector.findTags(frame, gray, edges, thresh_val, thresh2_val, blur_val);

		// show locations of tags in window
//		std::cout << "Found " << tags.size() << " tags" << std::endl;
		
		// draw lines between tag points
		// draw markers on the center and corners
		for (AR::Tag tag : tags)
		{
			std::cout << "Tag ID: " << tag.getID() << std::endl;
			std::vector<cv::Point> corners = tag.getCorners();
			corners.push_back(corners[0]);
			for (int i = 0; i < corners.size() - 1; i++)
			{
				//   cv::line(frame, corners[i].point, corners[i + 1].point, cv::Scalar(0, 0, 255),
				//            3);
				//   double l = std::sqrt(pow(corners[i].point.x - corners[i + 1].point.x, 2) +
				//                        pow(corners[i].point.y - corners[i + 1].point.y, 2));
				//   cv::drawMarker(frame, corners[i].point, cv::Scalar(255, 0, 0),
				//                  cv::MARKER_CROSS, l / 10, 2, cv::FILLED);
			}
			cv::drawMarker(frame, tag.getCenter(), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 15,
			               2, cv::FILLED);
			std::vector<cv::Point2d> cubePoints = projectCube(200, tag.getRVec(), tag.getTVec());
			// std::cout << "rvec: " << tag.getRVec() << std::endl
			//          << "tvec: " << tag.getTVec() << std::endl;
			// std::cout << "Normal vector: " << normal << std::endl;
			//cv::line(frame, cubePoints[0], cubePoints[1], cv::Scalar(0, 0, 255), 3);
			for(size_t i = 0; i < 4; i++){
				size_t next = (i==3 ? 0 : i+1);
				cv::line(frame, cubePoints[i], cubePoints[next], cv::Scalar(0,0,255), 3);
				cv::line(frame, cubePoints[i], cubePoints[i+4], cv::Scalar(0,255,0), 3);
				cv::line(frame, cubePoints[i+4], cubePoints[next+4], cv::Scalar(255,0,0), 3);
			}
		}

		cv::imshow(ORIG_WINDOW_NAME, frame);
		if (EXTRA_WINDOWS)
		{
			cv::imshow("Gray", gray);
			cv::imshow("Edges", edges);
		}

		if (cv::waitKey(5) == 'q')
			break;


		std::clock_t c_end = std::clock();

		total_time += 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
		std::cout << "Average CPU time used: " << total_time / loop_num << " ms\n";
	}
	return 0;
}
