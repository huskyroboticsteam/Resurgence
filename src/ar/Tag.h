#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include <vector>
#include <chrono>
#include <iostream>

#define EVAN_LAPTOP_CAMERA_PARAMS 1
#define EVAN_WEBCAM_PARAMS 2
#define ZED_CAMERA_PARAMS 3

// Change to select which set of intrinsic camera parameters to use.
#define USE_CAMERA_PARAMS EVAN_LAPTOP_CAMERA_PARAMS

namespace AR
{
	/*
	  Camera parameters
	  --------------------
	  These are obtained from camera calibration and are unique to and constant for
	  every camera. Change the macro above to use a different set of camera parameters.
	*/

#ifndef USE_CAMERA_PARAMS
#error "USE_CAMERA_PARAMS macro is not defined"
#elif USE_CAMERA_PARAMS == EVAN_LAPTOP_CAMERA_PARAMS
	// Parameters for Evan's laptop camera:
	const double CAMERA_PARAMS_ARRAY[9] = {5.6534586568739849e+02, 0., 3.5588060437029822e+02,
										   0.,5.6534586568739849e+02, 2.9541081641775287e+02,
										   0., 0., 1.};
	const double DISTORTION_PARAMS_ARRAY[5] = {3.4425270669197583e-01, -4.5627505509852968e+00,
											   1.4729564760154447e-04, -1.2416402052553264e-02,
											   1.5720190712074883e+01};
#elif USE_CAMERA_PARAMS == EVAN_WEBCAM_PARAMS

	//Parameters for Evan's webcam:
	const double CAMERA_PARAMS_ARRAY[9] = {5.5929841506515140e+02, 0., 3.3632998221165434e+02,
										   0.,5.5929841506515140e+02, 2.4560280586167062e+02,
										   0., 0., 1.};
	const double DISTORTION_PARAMS_ARRAY[5] = {2.2319725483552814e-02, -2.3138101190867111e-01,
											   3.6220766734074462e-03, 3.8852893952725500e-03,
											   5.4773015987500950e-01};
#else
#error "Unknown value for USE_CAMERA_PARAMS"
#endif

	// Matrices constructed from Camera and Distortion Parameters
	const cv::Mat CAMERA_PARAMS(3,3,CV_64FC1,*CAMERA_PARAMS_ARRAY);
	const cv::Mat DISTORTION_PARAMS(5,1,CV_64FC1,*DISTORTION_PARAMS_ARRAY);
	
	struct Corner
	{
		double angle;
		cv::Point point;
	};

	class InvalidCornerException: public std::exception
	{
		virtual const char* what() const throw();
	};

	enum CornerIndex{
		TOP_LEFT = 0,
		TOP_RIGHT= 1,
		BOTTOM_RIGHT = 2,
		BOTTOM_LEFT = 3
	};
	
	class Tag
	{
	private:
		std::vector<Corner> corners;
		cv::Vec3d orientation;
		cv::Vec3d calcOrientation();
	public:
		Tag(cv::Point top_left,
			cv::Point top_right,
			cv::Point bottom_right,
			cv::Point bottom_left);
		cv::Point getCenter() const;
		std::vector<Corner> getCorners() const;
		float getPitch() const;
		float getYaw() const;
		float getRoll() const;
		float getDistance();
	};
}
