#include "CameraParams.h"

namespace AR
{
    CameraParams::CameraParams(cv::Mat camera_params, cv::Mat dist_coeff)
	{
        this->camera_params = camera_params;
        this->dist_coeff = dist_coeff;
    }

    cv::Mat CameraParams::getCameraParams()
	{
        return camera_params;
    }

    cv::Mat CameraParams::getDistCoeff()
	{
        return dist_coeff;
    }
}
