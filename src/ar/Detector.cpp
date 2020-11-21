#include "Detector.h"

#include <cassert>
#include <memory>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "CameraParams.h"
#include "Tag.h"

namespace AR
{

template <class MarkerName_t>
Detector<MarkerName_t>::Detector(std::shared_ptr<MarkerSet<MarkerName_t>> marker_set,
								 CameraParams camera_params,
								 cv::Ptr<cv::aruco::DetectorParameters> detector_params)
	: marker_set_(marker_set), camera_params_(camera_params), detector_params_(detector_params)
{
	assert(marker_set != nullptr);
	assert(detector_params != nullptr);
}

template <class MarkerName_t>
std::vector<Tag> Detector<MarkerName_t>::detectTags(const cv::Mat &input)
{
	std::vector<std::vector<cv::Point2d>> corners;
	std::vector<int> ids;
	cv::aruco::detectMarkers(input, this->marker_set_->getDict(), corners, ids,
							 this->detector_params_);

	std::vector<cv::Vec3d> rvecs, tvecs;
	cv::aruco::estimatePoseSingleMarkers(corners, this->marker_set_->getPhysicalSize(),
										 this->camera_params_.getCameraParams(),
										 this->camera_params_.getDistCoeff(), rvecs, tvecs);

	std::vector<Tag> tags;
	for(size_t i = 0; i < ids.size(); i++){
		int id = ids[i];
		Marker marker = this->marker_set_->getMarkers()[id];
		Tag current(marker, rvecs[i], tvecs[i]);
		tags.push_back(current);
	}

	return tags;
}

} // namespace AR
