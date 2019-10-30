#pragma once

#include "Tag.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <vector>

using namespace std;

namespace AR
{
	//new functions
	std::vector<Tag> findTags(cv::Mat input,
							  int canny_thresh_1 = 50,
							  int canny_thresh_2 = 50,
							  int blur_size = 5);
}
