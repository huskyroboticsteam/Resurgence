#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include "CameraParams.h"
#include "MarkerSet.h"
#include "Tag.h"

namespace AR
{

// TODO: determine if we need this or restructure it
struct DetectorOutput
{
	cv::Mat grayscale_mat;
	cv::Mat edges_mat;
	std::vector<cv::Mat> tag_views;
	std::vector<std::vector<cv::Point2f>> rejected_corners;
};

class Detector
{
private:
	std::shared_ptr<MarkerSet> marker_set_;
	CameraParams camera_params_;
	cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

public:
	Detector(std::shared_ptr<MarkerSet> marker_set, CameraParams camera_params,
			 cv::Ptr<cv::aruco::DetectorParameters> detector_params =
				 cv::aruco::DetectorParameters::create());
	std::vector<Tag> detectTags(const cv::Mat &input);
};

} // namespace AR
