#pragma once

#include <opencv2/core.hpp>

namespace AR
{
/**
  Camera parameters are obtained from camera calibration and are unique to and constant for
  every camera.
*/
class CameraParams
{
	private:
		cv::Mat _camera_params;
		cv::Mat _dist_coeff;
	public:
		CameraParams(cv::Mat camera_params, cv::Mat dist_coeff);
		cv::Mat getCameraParams() const;
		cv::Mat getDistCoeff() const;
		static CameraParams DEFAULT;
		static CameraParams WEBCAM;
		static CameraParams LAPTOP;
};
}
