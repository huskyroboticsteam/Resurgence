#include "Detector.h"
#include "ThreadedCapture.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <ctime>
#include <chrono>

const std::string ORIG_WINDOW_NAME = "Image (Original)";
const std::string THRESH_TRACKBAR_NAME = "Threshold";
const std::string THRESH2_TRACKBAR_NAME = "Threshold 2";
const std::string BLUR_TRACKBAR_NAME = "Blur";

constexpr bool EXTRA_WINDOWS = false;

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
	AR::ThreadedCapture cap;

	int api_id = cv::CAP_ANY;

	int thresh_val = 50;
	int thresh2_val = 120;
	int blur_val = 2;

	std::cout << "Opening camera..." << std::endl;

	cap.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	
	if (!cap.open(camera_id + api_id))
	{
		std::cerr << "ERROR! Unable to open camera" << std::endl;
		return 1;
	}

	std::cout << "Opening image window, press Q to quit" << std::endl;

	cv::namedWindow(ORIG_WINDOW_NAME);
	cv::createTrackbar(THRESH_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh_val, 256);
	cv::createTrackbar(THRESH2_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh2_val, 256);
	cv::createTrackbar(BLUR_TRACKBAR_NAME, ORIG_WINDOW_NAME, &blur_val, 10);

	AR::Detector detector(PARAMS);

	double loop_num = 0.0;
	long double total_time = 0.0;
	long double total_wall = 0.0;

	int count = 0;
	char choice = ' ';

	std::cout << "1) Continuously grab frame. \n2) Grab 10 frames at a time" << std::endl;
	std::cin >> choice;

	bool frame_by_frame = choice == '2';

	while (true)
	{	
		if (frame_by_frame && count % 10 == 0)
		{
			std::cin.get();
		}

		std::clock_t c_start = std::clock(); // Stores current cpu time
		auto wall_start = std::chrono::system_clock::now();
		loop_num++;

		// Grabs frame
		auto read_start = std::chrono::system_clock::now();
		cap.read(frame);
		auto read_end = std::chrono::system_clock::now();
		if (frame.empty())
		{
			std::cerr << "ERROR! Blank frame grabbed" << std::endl;
			break;
		}
		long double read_time = std::chrono::duration_cast<std::chrono::milliseconds>
			(read_end - read_start).count();
		std::cout << "Read took " << read_time << "ms" <<std::endl;

		// Mats to hold grayscale and edge detection results
		cv::Mat gray;
		cv::Mat edges;

		std::vector<std::vector<cv::Point2f> > quad_corners;

		// Passes frame to the detector class. 
		// Tags will be located and returned.
		std::vector<AR::Tag> tags =
		    detector.findTags(frame, gray, edges, quad_corners, thresh_val, thresh2_val, blur_val);

		// Draws a green line around all quadrilateral shapes found in the image (debugging purposes)
		cv::Scalar green_line = cv::Scalar(0, 255, 0);
		for (std::vector<cv::Point2f> quad: quad_corners)
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
			std::cout << "Tag ID: " << tag.getID() << std::endl;
			std::vector<cv::Point> corners = tag.getCorners();
			corners.push_back(corners[0]);
			cv::drawMarker(frame, tag.getCenter(), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 15,
			               2, cv::FILLED);
			std::vector<cv::Point2d> cubePoints = projectCube(0.2, tag.getRVec(), tag.getTVec());
			std::cout << "rvec: " << tag.getRVec() << std::endl;
			std::cout << "tvec: " << tag.getTVec() << std::endl;
			std::cout << "coordinates: " << tag.getCoordinates() << std::endl;

			for(size_t i = 0; i < 4; i++){
				size_t next = (i==3 ? 0 : i+1);
				cv::line(frame, cubePoints[i], cubePoints[next], cv::Scalar(0,0,255), 3);
				cv::line(frame, cubePoints[i], cubePoints[i+4], cv::Scalar(0,255,0), 3);
				cv::line(frame, cubePoints[i+4], cubePoints[next+4], cv::Scalar(255,0,0), 3);
			}
		}

		// Opens up a window to display frames
		cv::imshow(ORIG_WINDOW_NAME, frame);
		if (EXTRA_WINDOWS)
		{
			cv::imshow("Gray", gray);
			cv::imshow("Edges", edges);
		}

		if (cv::waitKey(1) == 'q')
			break;

		// Calculates time used to complete one loop on average
		std::clock_t c_end = std::clock();
		auto wall_end = std::chrono::system_clock::now();
		auto wall_t = std::chrono::duration_cast<std::chrono::microseconds>(wall_end - wall_start)
			.count() / 1000.0;
		long double cpu_t = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
		total_time += cpu_t;
		total_wall += wall_t;
		std::cout << "CPU Time: " << cpu_t << "ms, avg: " << total_time / loop_num
				  << "ms; Wall Time: " << wall_t << "ms, avg: " << total_wall / loop_num
				  << "ms" << std::endl;
	}
	
	return 0;
}
