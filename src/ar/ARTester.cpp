#include "Detector.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace std;

const string WINDOW_NAME = "Image";

int main()
{
    cv::Mat frame;
    cv::VideoCapture cap;

    int device_id = 0;
    int api_id = cv::CAP_ANY;

    cout << "Opening camera..." << endl;
    cap.open(device_id + api_id);
    if(!cap.isOpened())
	{
		cerr << "ERROR! Unable to open camera" << endl;
        return 1;
	}

	cout << "Opening image window, press Q to quit" << endl;
	cv::namedWindow(WINDOW_NAME);
    while(true)
	{
		cap.read(frame);
		if(frame.empty())
		{
			cerr << "ERROR! Blank frame grabbed" << endl;
			break;
		}

		//here is ideally where we'd pass the frame to the detector
		
		cv::imshow(WINDOW_NAME, frame);

		if(cv::waitKey(5) == 'q')
			break;
	}
	return 0;
}
