#pragma once

#include <opencv2/core.hpp>

#include "Marker.h"

namespace AR
{

/**
   @brief Class representing an AR tag detected in an image.

   This class is simply used as a way to represent the result of a detection, and can
   provide location and orientation information. You should likely never need to construct
   a Tag yourself; this is done internally by the detection code. All tags are immutable
   after construction.
 */
class Tag
{
private:
	cv::Vec3d rvec_;
	cv::Vec3d tvec_;
	MarkerPattern marker_;

public:
	Tag(MarkerPattern marker, cv::Vec3d rvec, cv::Vec3d tvec);
	Tag() = delete;
	/**
	   Gets the rotation vector of the tag. This vector defines a line through the origin
	   around which points are rotated to transform them from the object reference frame to the
	   camera reference frame; its length is the angle of rotation. You will likely not need
	   this for most uses.
	 */
	cv::Vec3d getRVec() const;
	/**
	   Gets the translation vector of the tag. This vector defines the translation necessary to
	   transform points from the object reference frame to the camera reference frame. For the
	   center of the tag (0,0,0), this vector is in effect the 3D coordinates of the tag
	   relative to the camera.

	   This function will return real world units; these are dependent upon the camera
	   parameters you are currently using.
	 */
	cv::Vec3d getTVec() const;
	/**
	   Gets the 3D coordinates of the tag with respect to the camera. The return value is
	   currently the same as getTVec, but this function is provided for convenience as its name
	   is more intuitive.

	   This function will return real world units; these are dependent upon the camera
	   parameters you are currently using.
	 */
	cv::Vec3d getCoordinates() const;
	/**
	   Gets the Marker associated with the tag.
	 */
	MarkerPattern getMarker() const;
};
} // namespace AR
