#include "Detector.h"

namespace AR
{
	const int CONTOUR_AREA_THRESH = 1000;
    cv::Mat removeNoise(cv::Mat input, int blur_size=5)
    {
		cv::Mat blur;
		int bsize = (blur_size*2)+1;
		cv::GaussianBlur(input, blur, cv::Size(bsize,bsize), 0);
		//FIXME: Change to bilateral blur or some other faster method.
		return blur;
    }

	cv::Mat prepImage(cv::Mat input, int blur_size, int thresh_val, int thresh_val2)
    {
		cv::Mat blur = removeNoise(input, blur_size);
        cv::Mat gray;
        cv::Mat edges;
		
        cv::cvtColor(blur, gray, cv::COLOR_RGB2GRAY);
        cv::Canny(gray, edges, thresh_val, thresh_val2);
		cv::dilate(edges, edges, cv::Mat(), cv::Point(-1,-1));
        return edges;
    }

	std::vector<Tag> findTags(cv::Mat input,
							  int canny_thresh_1,
							  int canny_thresh_2,
							  int blur_size)
	{

		cv::Mat prepped_input = prepImage(input, blur_size, canny_thresh_1, canny_thresh_2);
		
		std::vector<std::vector<cv::Point>> contours;
		std::vector<Tag> output;
		
		cv::findContours(prepped_input, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

		for(size_t i = 0; i < contours.size(); i++)
		{
			std::vector<cv::Point> approx;
			cv::approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
			if(approx.size() == 4
			   && cv::isContourConvex(approx)
			   && cv::contourArea(approx) > CONTOUR_AREA_THRESH)
			{
				//make Tag with approximated coordinates
				Tag tag(approx[0], approx[1], approx[2], approx[3]);
				output.push_back(tag);
			}
		}
		
		return output;
	}
} // namespace AR
