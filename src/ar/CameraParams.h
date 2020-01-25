#pragma once

#include <opencv2/core.hpp>

namespace AR
{
    class CameraParams
	{
        private:
            cv::Mat _camera_params;
            cv::Mat _dist_coeff;
        public:
            CameraParams(cv::Mat camera_params, cv::Mat dist_coeff);
			static CameraParams DEFAULT;
            cv::Mat getCameraParams() const;
            cv::Mat getDistCoeff() const;
    };
}
