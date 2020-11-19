#include "Tag.h"

#include <cassert>
#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>

#include "../Util.h"

namespace AR
{

Tag::Tag(Marker marker, cv::Vec3d rvec, cv::Vec3d tvec)
	: marker_(marker), rvec_(rvec), tvec_(tvec){}

cv::Vec3d Tag::getRVec() const
{
	return rvec_;
}

cv::Vec3d Tag::getTVec() const
{
	return tvec_;
}

cv::Vec3d Tag::getCoordinates() const
{
	return tvec_;
}

Marker Tag::getMarker() const
{
	return marker_;
}

} // namespace AR
