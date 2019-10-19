#pragma once

#include "Tag.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

using namespace std;

namespace AR
{
	const int CONTOUR_AREA_THRESH = 1000;
	cv::Mat PrepImage(cv::Mat input, int thresh_val=0, int thresh_val2=128);
	vector<vector<cv::Point>> FindCandidates(cv::Mat input);

	//new functions
	std::vector<Tag> findTags(cv::Mat input);
}
