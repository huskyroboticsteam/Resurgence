#pragma once
#include <opencv2/core.hpp>

namespace AR
{
    class CameraParams
	{
        private:
            cv::Mat camera_params;
            cv::Mat dist_coeff;
        public:
            CameraParams(cv::Mat, cv::Mat);
            cv::Mat getCameraParams();
            cv::Mat getDistCoeff();
    };
}
