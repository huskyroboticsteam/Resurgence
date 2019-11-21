#include "Detector.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

const std::string ORIG_WINDOW_NAME = "Image (Original)";
const std::string THRESH_TRACKBAR_NAME = "Threshold";
const std::string THRESH2_TRACKBAR_NAME = "Threshold 2";
const std::string BLUR_TRACKBAR_NAME = "Blur";

constexpr bool EXTRA_WINDOWS = true;

int camera_id = 0;

int main(int argc, char* argv[])
{
	if(argc > 1)
	{
		camera_id = std::stoi(argv[1]);
	}

	
	cv::Mat frame;
	cv::VideoCapture cap;

	int api_id = cv::CAP_ANY;

	int thresh_val = 50;
	int thresh2_val = 50;
	int blur_val = 2;

	std::cout << "Opening camera..." << std::endl;

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

	cap.open(camera_id + api_id);
	if (!cap.isOpened())
	{
		std::cerr << "ERROR! Unable to open camera" << std::endl;
		return 1;
	}

	std::cout << "Opening image window, press Q to quit" << std::endl;

	cv::namedWindow(ORIG_WINDOW_NAME);
	cv::createTrackbar(THRESH_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh_val, 256);
	cv::createTrackbar(THRESH2_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh2_val, 256);
	cv::createTrackbar(BLUR_TRACKBAR_NAME, ORIG_WINDOW_NAME, &blur_val, 10);

	while (true)
	{
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
		    AR::findTags(frame, gray, edges, thresh_val, thresh2_val, blur_val);

		// show locations of tags in window
		std::cout << "Found " << tags.size() << " tags" << std::endl;

		// draw lines between tag points
		// draw markers on the center and corners
		for (AR::Tag tag : tags)
		{
			std::vector<AR::Corner> corners = tag.getCorners();
			corners.push_back(corners[0]);
			for (int i = 0; i < corners.size() - 1; i++)
			{
				cv::line(frame, corners[i].point, corners[i + 1].point, cv::Scalar(0, 0, 255),
				         3);
				cv::drawMarker(frame, corners[i].point, cv::Scalar(255, 0, 0),
				               cv::MARKER_CROSS, 70, 6, cv::FILLED);
			}
			cv::drawMarker(frame, tag.getCenter(), cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 70,
			               6, cv::FILLED);
			std::cout << tag.getCenter() << ", ";
		}

		std::cout << std::endl;

		cv::imshow(ORIG_WINDOW_NAME, frame);
		if (EXTRA_WINDOWS)
		{
			cv::imshow("Gray", gray);
			cv::imshow("Edges", edges);
		}

		if (cv::waitKey(5) == 'q')
			break;
	}
	return 0;
}
