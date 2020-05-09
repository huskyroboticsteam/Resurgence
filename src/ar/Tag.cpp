#include "Tag.h"

#include <cassert>
#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>

#include "../Util.h"

namespace AR
{

Tag::Tag(cv::Point top_left, cv::Point top_right, cv::Point bottom_right,
		 cv::Point bottom_left, CameraParams params, TagID tag_id)
	: params(params)
{
	// validate points
	// TODO determine if checkcorners is actually necessary
	//	checkCorners(top_left, top_right, bottom_right, bottom_left);

	// fill vector with points
	corners.push_back(top_left);
	corners.push_back(top_right);
	corners.push_back(bottom_right);
	corners.push_back(bottom_left);

	id = tag_id;

	calcOrientation();
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
		tri_centers.push_back(getTriCenter(corners[prev], corners[i], corners[next]));
	}
	return getQuadCenter(tri_centers[0], tri_centers[1], tri_centers[2], tri_centers[3]);
}

std::vector<cv::Point> Tag::getCorners() const
{
	return corners;
}

void Tag::calcOrientation()
{
	// vectors to hold image points (detected corners in image) and object points ("ideal"
	// corners of the tag in the world coordinate system)
	std::vector<cv::Point2f> image_points;
	std::vector<cv::Point3f> object_points;

	// add detected corners to image points vector
	for (int i = 0; i < corners.size(); i++)
	{
		image_points.push_back(corners[i]);
	}

	// create ideal object points
	double square_len = 0.2;
	// top left (-w/2, w/2)
	object_points.push_back(cv::Point3f(-(square_len / 2), (square_len / 2), 0));
	// top right (w/2, w/2)
	object_points.push_back(cv::Point3f((square_len / 2), (square_len / 2), 0));
	// bottom right (w/2, -w/2)
	object_points.push_back(cv::Point3f((square_len / 2), -(square_len / 2), 0));
	// bottom left (-w/2, -w/2)
	object_points.push_back(cv::Point3f(-(square_len / 2), -(square_len / 2), 0));

	// std::cout << "object points: " << object_points << std::endl;

	// Mat objects to hold returned rotation and translation vectors
	cv::Mat _rvec;
	cv::Mat _tvec;

	// estimate pose
	cv::solvePnP(object_points, image_points, params.getCameraParams(), params.getDistCoeff(),
				 _rvec, _tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

	// store rotation and translation vectors in this tag instance
	rvec = _rvec;
	tvec = _tvec;
}

TagID Tag::getID() const
{
	return this->id;
}

cv::Vec3d Tag::getRVec() const
{
	return rvec;
}

cv::Vec3d Tag::getTVec() const
{
	return tvec;
}

cv::Vec3d Tag::getCoordinates() const
{
	return tvec;
}

} // namespace AR
