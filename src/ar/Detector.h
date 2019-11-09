#pragma once

#include "Tag.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vector>

using namespace std;

namespace AR
{
	std::vector<Tag> findTags(cv::Mat input,
							  cv::Mat& grayscale,
							  cv::Mat& edges,
							  int canny_thresh_1 = 50,
							  int canny_thresh_2 = 50,
							  int blur_size = 5);

	std::vector<Tag> findTags(cv::Mat input,
							  int canny_thresh_1 = 50,
							  int canny_thresh_2 = 50,
							  int blur_size = 5);
}
