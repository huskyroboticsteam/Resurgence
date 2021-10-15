#include "Tag.h"

namespace AR {

Tag::Tag(MarkerPattern marker, cv::Vec3d rvec, cv::Vec3d tvec)
	: rvec_(rvec), tvec_(tvec), marker_(marker) {
}

cv::Vec3d Tag::getRVec() const {
	return rvec_;
}

cv::Vec3d Tag::getTVec() const {
	return tvec_;
}

cv::Vec3d Tag::getCoordinates() const {
	return tvec_;
}

MarkerPattern Tag::getMarker() const {
	return marker_;
}

} // namespace AR
