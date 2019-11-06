#include "Detector.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace std;

const string ORIG_WINDOW_NAME = "Image (Original)";
const string THRESH_TRACKBAR_NAME = "Threshold";
const string THRESH2_TRACKBAR_NAME = "Threshold 2";
const string BLUR_TRACKBAR_NAME = "Blur";

int main()
{
    cv::Mat frame;
    cv::VideoCapture cap;

    int device_id = 0;
    int api_id = cv::CAP_ANY;

	int thresh_val = 128;
	int thresh2_val = 128;
	int blur_val = 2;

    cout << "Opening camera..." << endl;

	cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
	
    cap.open(device_id + api_id);
    if(!cap.isOpened())
	{
		cerr << "ERROR! Unable to open camera" << endl;
        return 1;
	}

	cout << "Opening image window, press Q to quit" << endl;

	cv::namedWindow(ORIG_WINDOW_NAME);
	cv::createTrackbar(THRESH_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh_val, 256);
	cv::createTrackbar(THRESH2_TRACKBAR_NAME, ORIG_WINDOW_NAME, &thresh2_val, 256);
	cv::createTrackbar(BLUR_TRACKBAR_NAME, ORIG_WINDOW_NAME, &blur_val, 10);
	
    while(true)
	{
		cap.read(frame);
		if(frame.empty())
		{
			cerr << "ERROR! Blank frame grabbed" << endl;
			break;
		}

		// pass frame to detector here
		std::vector<AR::Tag> tags = AR::findTags(frame, thresh_val, thresh2_val, blur_val);

		// show locations of tags in window
		std::cout << "Found " << tags.size() << " tags" << std::endl;
		
		// draw lines between tag points
		for(AR::Tag tag : tags)
		{
			std::vector<AR::Corner> corners = tag.getCorners();
			corners.push_back(corners[0]);
			std::cout << "Drawing lines between: ";
			for(int i = 0; i < corners.size()-1; i++)
			{
				std::cout << corners[i].point << " and " << corners[i+1].point << ", ";
				cv::line(frame, corners[i].point, corners[i + 1].point, cv::Scalar(0, 255, 0));
			}
			std::cout << std::endl;
        }

        cv::imshow(ORIG_WINDOW_NAME, frame);

		if(cv::waitKey(5) == 'q')
			break;
	}
	return 0;
}
