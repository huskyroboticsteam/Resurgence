#pragma once

#include <memory>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

#include "../camera/CameraParams.h"
#include "MarkerSet.h"
#include "Tag.h"

namespace AR
{

/**
   @addtogroup ar
   @{
 */
class Detector
{
private:
	std::shared_ptr<MarkerSet> marker_set_;
	cam::CameraParams camera_params_;
	cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

public:
	Detector(std::shared_ptr<MarkerSet> marker_set, cam::CameraParams camera_params,
			 cv::Ptr<cv::aruco::DetectorParameters> detector_params =
				 cv::aruco::DetectorParameters::create());
	std::vector<Tag> detectTags(const cv::Mat &input,
								std::vector<std::vector<cv::Point2f>> &rejected_points);
	std::vector<Tag> detectTags(const cv::Mat &input);
	cv::aruco::DetectorParameters getDetectorParams();
	void setDetectorParams(cv::aruco::DetectorParameters params);
};
/** @} */

} // namespace AR
