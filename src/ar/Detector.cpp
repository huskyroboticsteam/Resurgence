#include "Detector.h"

#include <cassert>
#include <memory>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "../camera/CameraParams.h"
#include "Tag.h"

namespace AR
{

Detector::Detector(std::shared_ptr<MarkerSet> marker_set,
				   cam::CameraParams camera_params,
				   cv::Ptr<cv::aruco::DetectorParameters> detector_params)
	: marker_set_(marker_set), camera_params_(camera_params), detector_params_(detector_params)
{
	assert(marker_set != nullptr);
	assert(detector_params != nullptr);
	this->detector_params_->markerBorderBits = marker_set->getBorderSize();
	this->detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

std::vector<Tag> Detector::detectTags(const cv::Mat &input){
	std::vector<std::vector<cv::Point2f>> junk;
	return detectTags(input, junk);
}

std::vector<Tag> Detector::detectTags(const cv::Mat &input,
									  std::vector<std::vector<cv::Point2f>> &rejectedPoints){
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;
	cv::aruco::detectMarkers(input, this->marker_set_->getDict(), corners, ids,
							 this->detector_params_,
							 rejectedPoints,
							 this->camera_params_.getCameraMatrix(),
							 this->camera_params_.getDistCoeff());
	int id_count = ids.size();

	std::vector<cv::Vec3d> rvecs, tvecs;
	cv::aruco::estimatePoseSingleMarkers(corners, this->marker_set_->getPhysicalSize(),
										 this->camera_params_.getCameraMatrix(),
										 this->camera_params_.getDistCoeff(), rvecs, tvecs);

	std::vector<Tag> tags;
	for(size_t i = 0; i < ids.size(); i++){
		int id = ids[i];
		MarkerPattern marker = this->marker_set_->getMarkers()[id];
		Tag current(marker, rvecs[i], tvecs[i]);
		tags.push_back(current);
	}

	return tags;
}

} // namespace AR
