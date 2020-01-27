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

CameraParams CameraParams::DEFAULT(cv::Mat::eye(3,3,CV_64F),
								   cv::Mat::zeros(3,3,CV_64F));

CameraParams CameraParams::WEBCAM((cv::Mat_<double>(3,3)
								   << 5.5929841506515140e+02, 0., 3.3632998221165434e+02,
								   0.,5.5929841506515140e+02, 2.4560280586167062e+02,
								   0., 0., 1.),
								  (cv::Mat_<double>(5,1)
								   << 2.2319725483552814e-02, -2.3138101190867111e-01,
								   3.6220766734074462e-03, 3.8852893952725500e-03,
								   5.4773015987500950e-01));

CameraParams CameraParams::LAPTOP((cv::Mat_<double>(3,3)
								   << 5.6534586568739849e+02, 0., 3.5588060437029822e+02,
								   0.,5.6534586568739849e+02, 2.9541081641775287e+02,
								   0., 0., 1.),
								  (cv::Mat_<double>(5,1)
								   << 3.4425270669197583e-01, -4.5627505509852968e+00,
								   1.4729564760154447e-04, -1.2416402052553264e-02,
								   1.5720190712074883e+01));

}
