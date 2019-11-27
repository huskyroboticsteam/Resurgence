#include "Tag.h"

#include <catch2/catch.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

#include <iostream>
#include <cmath>

TEST_CASE("square tag has zero angles", "[tag]")
{
	std::vector<cv::Point3f> undistorted;
	double w = 0.5;
	double h = 0.5;
	double cx = 0;
	double cy = 0;
	undistorted.push_back(cv::Point3f(cx-(w/2), cy-(h/2), 0));
	undistorted.push_back(cv::Point3f(cx+(w/2), cy-(h/2), 0));
	undistorted.push_back(cv::Point3f(cx+(w/2), cy+(h/2), 0));
	undistorted.push_back(cv::Point3f(cx-(w/2), cy+(h/2), 0));
	
	std::vector<cv::Point2f> distorted;
	cv::Vec3d rvec(0, 0, 0);
	cv::Vec3d tvec(0, 0, 0);
	cv::projectPoints(undistorted, rvec, tvec, AR::CAMERA_PARAMS, AR::DISTORTION_PARAMS,
	                  distorted);
	std::cout << "distorted: " << distorted << std::endl;

	AR::Tag tag(distorted[0], distorted[1], distorted[2], distorted[3]);
	REQUIRE(tag.getPitch() == Approx(0.0).margin(1e-6));
	REQUIRE(tag.getYaw() == Approx(0.0).margin(1e-6));
	REQUIRE(tag.getRoll() == Approx(0.0).margin(1e-6));
}
