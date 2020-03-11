#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "CameraParams.h"

namespace AR
{

/**
   @brief Enum containing values for the corner indices.

   This is mostly used internally but may be used as a reference to see what the correct
   corner ordering is. The order currently goes clockwise starting with top left.
 */
enum CornerIndex
{
	TOP_LEFT = 0,
	TOP_RIGHT = 1,
	BOTTOM_RIGHT = 2,
	BOTTOM_LEFT = 3
};

/**
   Tag ID enum. Each member's value is the numerical ID of the tag it stands for.
 */
enum TagID
{
	ID_UNKNOWN = -1,
	ID_LEG_1 = 0,
	ID_LEG_2 = 10,
	ID_LEG_3 = 11,
	ID_LEG_4L = 1,
	ID_LEG_4R = 2,
	ID_LEG_5L = 8,
	ID_LEG_5R = 9,
	ID_LEG_6L = 3,
	ID_LEG_6R = 12,
	ID_LEG_7L = 6,
	ID_LEG_7R = 7
};

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
	/** Stores the corners of the tag in the image */
	std::vector<cv::Point> corners;
	cv::Vec3d rvec;
	cv::Vec3d tvec;
	TagID id;
	CameraParams params;

	/** Calculates the orientation */
	void calcOrientation();

public:
	/**
	   Constructor for a Tag. Takes four OpenCV points which are the coordinates of the
	   corners of the Tag in the image. Note the order of points; this will be checked and
	   an AR::InvalidCornerException will be thrown if ordering is incorrect.

	   Also takes in a set of CameraParams, and the ID of this tag (the ID checking should be
	   done outside of the Tag class).

	   Please note that you should not need to call this constructor yourself as it is called
	   by the Detector class, with values determined through image processing that happens in
	   the Detector class.
	*/
	Tag(cv::Point top_left, cv::Point top_right, cv::Point bottom_right, cv::Point bottom_left,
		CameraParams params, TagID tag_id = ID_UNKNOWN);
	/** Gets the 2D (image) coordinates of the center of the tag. */
	cv::Point getCenter() const;
	/**
	   Gets the 2D (image) corners of the tag. Note that this is a const operation and a copy
	   of the corners will be returned.
	*/
	std::vector<cv::Point> getCorners() const;
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
	   Gets the ID of the tag. If the ID cannot be determined or is not a known ID marking a
	   leg of the autonomous course, the ID_UNKNOWN value will be returned.
	 */
	TagID getID() const;
};
} // namespace AR
