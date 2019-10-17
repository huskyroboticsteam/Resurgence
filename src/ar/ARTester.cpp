#include "Detector.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace std;

const string ORIG_WINDOW_NAME = "Image (Original)";
const string FILTER_WINDOW_NAME = "Image (Filtered)";
const string THRESH_TRACKBAR_NAME = "Threshold";
const string THRESH2_TRACKBAR_NAME = "Threshold 2";

int main()
{
    cv::Mat frame;
    cv::VideoCapture cap;

    int device_id = 0;
    int api_id = cv::CAP_ANY;

	int thresh_val = 128;
	int thresh2_val = 128;

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
	cv::namedWindow(FILTER_WINDOW_NAME);
	cv::createTrackbar(THRESH_TRACKBAR_NAME, FILTER_WINDOW_NAME, &thresh_val, 256);
	cv::createTrackbar(THRESH2_TRACKBAR_NAME, FILTER_WINDOW_NAME, &thresh2_val, 256);
	
    while(true)
	{
		cap.read(frame);
		if(frame.empty())
		{
			cerr << "ERROR! Blank frame grabbed" << endl;
			break;
		}
		
		cv::Mat new_frame = AR::PrepImage(frame, thresh_val, thresh2_val);
		vector<vector<cv::Point>> detected = AR::FindCandidates(new_frame);
		cv::drawContours(frame, detected, -1, cv::Scalar(0,0,255), 3);
		
		cv::imshow(FILTER_WINDOW_NAME, new_frame);
		cv::imshow(ORIG_WINDOW_NAME, frame);

		if(cv::waitKey(5) == 'q')
			break;
	}
	return 0;
}
