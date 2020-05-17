#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "CameraParams.h"
#include "Tag.h"

namespace AR
{
struct DetectorParams
{
	CameraParams camera_params = AR::Params::DEFAULT_PARAMS;
	int canny_thresh_1 = 50;
	int canny_thresh_2 = 120;
	int blur_size = 5;
	double tag_size = 200.0;
};

struct DetectorOutput
{
	cv::Mat grayscale_mat;
	cv::Mat edges_mat;
	std::vector<cv::Mat> tag_views;
	std::vector<std::vector<cv::Point2f>> rejected_corners;
};

std::vector<Tag> detectTags(cv::Mat input, DetectorParams params, DetectorOutput &output);

} // namespace AR
