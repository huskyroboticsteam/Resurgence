#include "Tag.h"
#include "../Util.h"

#include <opencv2/calib3d.hpp>

#include <cassert>
#include <iostream>
#include <cmath>

namespace AR
{
double findAngle(cv::Point a, cv::Point b, cv::Point c)
{
	a = a - b; // shift vectors so that B is at the origin
	b = b - b;
	c = c - b;
	double dot_product = a.ddot(c);
	double a_mag = sqrt((a.x * a.x) + (a.y * a.y));
	double c_mag = sqrt((c.x * c.x) + (c.y * c.y));
	return acos(dot_product / (a_mag * c_mag));
}

cv::Vec3d rotationMatrixToEulerAngles(cv::Mat &R)
{
	/*
	double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
	                 R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = util::almostEqual(sy, 0);

	double x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	*/
	double x,y,z;
	if(std::abs(R.at<double>(2,0)) != 1)
	{
		y = -asin(R.at<double>(2,0));
		x = atan2((R.at<double>(2,1)/cos(y)), (R.at<double>(2,2)/cos(y)));
		z = atan2((R.at<double>(1,0)/cos(y)), (R.at<double>(0,0)/cos(y)));
	}
	else 
	{
		z = 0;
		y = 0;
		if(R.at<double>(2,0) == -1)
		{
			x = atan2(R.at<double>(0,1), R.at<double>(0,2));
		}
		else
		{
			x = atan2(-R.at<double>(0,1), -R.at<double>(0,2));
		}
	}
	x = util::almostEqual(x, M_PI) ? M_PI-x : x;
	y = util::almostEqual(x, M_PI) ? M_PI-y : y;
	return cv::Vec3f(x, y, z);
}

void checkCorners(cv::Point top_left, cv::Point top_right, cv::Point bottom_right,
                  cv::Point bottom_left)
{
	// validate points
	assert(!(top_left.x - top_right.x > 0          // top left and top right are inverted
	         || top_left.y - bottom_left.y > 0     // top left and bottom left are inverted
	         || bottom_left.x - bottom_right.x > 0 // bottom left and bottom right are inverted
	         || top_right.y - bottom_right.y > 0)); // top right and bottom right are inverted
}

Tag::Tag(cv::Point top_left, cv::Point top_right, cv::Point bottom_right,
         cv::Point bottom_left)
{
	// validate points
	checkCorners(top_left, top_right, bottom_right, bottom_left);

	// fill vector with points
	std::vector<cv::Point> points;
	points.push_back(top_left);
	points.push_back(top_right);
	points.push_back(bottom_right);
	points.push_back(bottom_left);

	// turn points into Corners
	for (size_t i = 0; i < points.size(); i++)
	{
		size_t last_index = i == 0 ? points.size() - 1 : i - 1;
		size_t next_index = i == points.size() - 1 ? 0 : i + 1;
		double internal_angle = findAngle(points[last_index], points[i], points[next_index]);
		corners.push_back(Corner{internal_angle, points[i]});
	}
	orientation = calcOrientation();
}

cv::Point getTriCenter(cv::Point pt1, cv::Point pt2, cv::Point pt3)
{
	double centroidX = (pt1.x + pt2.x + pt3.x) / 3;
	double centroidY = (pt1.y + pt2.y + pt3.y) / 3;
	return cv::Point2d(centroidX, centroidY);
}

cv::Point getQuadCenter(cv::Point pt1, cv::Point pt2, cv::Point pt3, cv::Point pt4)
{
	double centroidX = (pt1.x + pt2.x + pt3.x + pt4.x) / 4;
	double centroidY = (pt1.y + pt2.y + pt3.y + pt4.y) / 4;
	return cv::Point2d(centroidX, centroidY);
}

cv::Point Tag::getCenter() const
{
	std::vector<cv::Point> tri_centers;
	for (size_t i = 0; i < 4; i++)
	{
		size_t next = (i == 3 ? 0 : i + 1);
		size_t prev = (i == 0 ? 3 : i - 1);
		tri_centers.push_back(
		    getTriCenter(corners[prev].point, corners[i].point, corners[next].point));
	}
	return getQuadCenter(tri_centers[0], tri_centers[1], tri_centers[2], tri_centers[3]);
}

std::vector<Corner> Tag::getCorners() const
{
	return corners;
}

cv::Vec3d Tag::calcOrientation()
{
	std::vector<cv::Point2f> image_points;
	std::vector<cv::Point3f> object_points;
	for (int i = 0; i < corners.size(); i++)
	{
		image_points.push_back(corners[i].point);
	}
	object_points.push_back(cv::Point3f(0, 0, 0));
	object_points.push_back(cv::Point3f(200, 0, 0));
	object_points.push_back(cv::Point3f(200, 200, 0));
	object_points.push_back(cv::Point3f(0, 200, 0));
	cv::Mat rvec;
	cv::Mat tvec;
	cv::solvePnP(object_points, image_points, CAMERA_PARAMS, DISTORTION_PARAMS, rvec, tvec);
	cv::Mat rmat;
	cv::Rodrigues(rvec, rmat);

	return rotationMatrixToEulerAngles(rmat);
}

float Tag::getPitch() const
{
	return orientation[0];
}

float Tag::getYaw() const
{
	return orientation[1];
}

float Tag::getRoll() const
{
	return orientation[2];
}

} // namespace AR
