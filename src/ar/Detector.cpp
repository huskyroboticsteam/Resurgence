#include "Detector.h"

#include <cassert>
#include <memory>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "../camera/CameraParams.h"
#include "Tag.h"

namespace AR
{

Detector::Detector(){}

Detector::Detector(std::shared_ptr<MarkerSet> marker_set,
				   cam::CameraParams camera_params,
				   cv::Ptr<cv::aruco::DetectorParameters> detector_params)
	: marker_set_(marker_set), camera_params_(camera_params), detector_params_(detector_params)
{
	if (!this->empty())
	{
		this->detector_params_->markerBorderBits = marker_set->getBorderSize();
		this->detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
		cv::initUndistortRectifyMap(camera_params_.getCameraMatrix(),
									camera_params_.getDistCoeff(), cv::Mat_<double>::eye(3, 3),
									camera_params_.getCameraMatrix(),
									camera_params_.getImageSize(), CV_16SC2, map1_, map2_);
	}
}

bool Detector::empty() const {
	return (marker_set_ == nullptr || camera_params_.empty() || detector_params_ == nullptr);
}

std::vector<Tag> Detector::detectTags(const cv::Mat &input){
	return _detectTagsImpl(input, nullptr);
}

std::vector<Tag> Detector::detectTags(const cv::Mat &input,
									  std::vector<std::vector<cv::Point2f>> &rejectedPoints){
	return _detectTagsImpl(input, &rejectedPoints);
}

std::vector<Tag> Detector::_detectTagsImpl(const cv::Mat &input,
										   std::vector<std::vector<cv::Point2f>>* rejected){
	if(this->empty()){
		return {};
	}
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> ids;
	cv::Mat undistorted;
	cv::remap(input, undistorted, this->map1_, this->map2_, cv::INTER_LINEAR);
	cv::aruco::detectMarkers(undistorted, this->marker_set_->getDict(), corners, ids,
							 this->detector_params_,
							 (rejected == nullptr ? cv::noArray() : *rejected),
							 this->camera_params_.getCameraMatrix(),
							 this->camera_params_.getDistCoeff());

	std::vector<cv::Vec3d> rvecs, tvecs;
	cv::aruco::estimatePoseSingleMarkers(corners, this->marker_set_->getPhysicalSize(),
										 this->camera_params_.getCameraMatrix(),
										 this->camera_params_.getDistCoeff(), rvecs, tvecs);
	std::vector<Tag> tags;
	for(size_t i = 0; i < ids.size(); i++){
		size_t id = static_cast<size_t>(ids[i]);
		MarkerPattern marker = this->marker_set_->getMarkers()[id];
		Tag current(marker, rvecs[i], tvecs[i]);
		tags.push_back(current);
	}

	return tags;
}

cv::aruco::DetectorParameters Detector::getDetectorParams(){
	return *detector_params_;
}

void Detector::setDetectorParams(cv::aruco::DetectorParameters params){
	*detector_params_ = params;
}

} // namespace AR
