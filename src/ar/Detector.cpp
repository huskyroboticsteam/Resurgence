#include "Detector.h"

using namespace std;

namespace AR
{
    cv::Mat RemoveNoise(cv::Mat input)
    {
        cv::Mat downscaled, upscaled;
        cv::pyrDown(input, downscaled, cv::Size(input.cols / 2, input.rows / 2));
        cv::pyrUp(downscaled, upscaled, input.size());
        return upscaled;
    }

	cv::Mat PrepImage(cv::Mat input, int thresh_val, int thresh_val2)
    {
        cv::Mat gray;
        cv::Mat thresh;
		
        cv::cvtColor(RemoveNoise(input), gray, cv::COLOR_RGB2GRAY);
        cv::Canny(gray, thresh, thresh_val, thresh_val2);
		cv::dilate(thresh, thresh, cv::Mat(), cv::Point(-1,-1));
        return thresh;
    }

	vector<vector<cv::Point>> FindCandidates(cv::Mat input)
	{
		vector<vector<cv::Point>> contours;
		vector<vector<cv::Point>> output;
		cv::findContours(input, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

		for(size_t i = 0; i < contours.size(); i++)
		{
			vector<cv::Point> approx;
			cv::approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
			if(approx.size() == 4
			   && cv::isContourConvex(approx)
			   && cv::contourArea(approx) > CONTOUR_AREA_THRESH)
			{
				output.push_back(approx);
			}
		}
		
		return output;
	}
} // namespace AR
