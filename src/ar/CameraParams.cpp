#include "CameraParams.h"

namespace AR
{

CameraParams::CameraParams(cv::Mat camera_params, cv::Mat dist_coeff)
{
	_camera_params = camera_params;
	_dist_coeff = dist_coeff;
}

cv::Mat CameraParams::getCameraParams() const
{
	return _camera_params;
}

cv::Mat CameraParams::getDistCoeff() const
{
	return _dist_coeff;
}
}
