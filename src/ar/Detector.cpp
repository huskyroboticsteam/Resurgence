#include "Detector.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace AR
{
const int CONTOUR_AREA_THRESH = 1000;
cv::Mat removeNoise(cv::Mat input, int blur_size = 5)
{
	cv::Mat blur;
	int bsize = (blur_size * 2) + 1;
	cv::GaussianBlur(input, blur, cv::Size(bsize, bsize), 0);
	// FIXME: Change to bilateral blur or some other faster method.
	return blur;
}

cv::Mat prepImage(cv::Mat input, int blur_size, int thresh_val, int thresh_val2,
                  cv::Mat &grayscale)
{
	cv::Mat blur = removeNoise(input, blur_size);
	cv::Mat gray;
	cv::Mat edges;

	cv::cvtColor(blur, gray, cv::COLOR_RGB2GRAY);
	cv::Canny(gray, edges, thresh_val, thresh_val2);
	cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1));

	grayscale = gray;

	return edges;
}

std::vector<Tag> findTags(cv::Mat input, cv::Mat &grayscale, cv::Mat &edges,
                          int canny_thresh_1, int canny_thresh_2, int blur_size)
{

	cv::Mat prepped_input =
	    prepImage(input, blur_size, canny_thresh_1, canny_thresh_2, grayscale);
	edges = prepped_input;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> heirarchy;
	std::vector<Tag> output;
	cv::findContours(prepped_input, contours, heirarchy, cv::RETR_TREE,
	                 cv::CHAIN_APPROX_SIMPLE);

	for (size_t i = 0; i < contours.size(); i++)
	{
		std::vector<cv::Point2f> approx;
		cv::approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);
		if (approx.size() == 4 && cv::isContourConvex(approx) &&
		    cv::contourArea(approx) > CONTOUR_AREA_THRESH)
		{
			// sort points into the correct order (top left, top right, bottom right,
			// bottom
			//  left)
			std::sort(approx.begin(), approx.end(),
			          [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.y < pt2.y; });
			std::sort(approx.begin(), approx.begin() + 2,
			          [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x < pt2.x; });
			std::sort(approx.begin() + 2, approx.end(),
			          [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x > pt2.x; });

			// make Tag with approximated coordinates
			// std::cout << "Found tag with corners " << approx << std::endl;
			Tag tag(approx[0], approx[1], approx[2], approx[3]);
			output.push_back(tag);
		}
	}

	return output;
}

std::vector<Tag> findTags(cv::Mat input, int canny_thresh_1, int canny_thresh_2, int blur_size)
{
	cv::Mat junk; // this Mat won't be used later. it's just used to be passed as a
	              // reference to the function this is overloading.
	return findTags(input, junk, junk, canny_thresh_1, canny_thresh_2, blur_size);
}
} // namespace AR
