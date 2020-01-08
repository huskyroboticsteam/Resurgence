#pragma once

#include <opencv2/core.hpp>

#include <vector>

#define EVAN_LAPTOP_CAMERA_PARAMS 1
#define EVAN_WEBCAM_PARAMS 2
#define ZED_CAMERA_PARAMS 3

// Change to select which set of intrinsic camera parameters to use.
#define USE_CAMERA_PARAMS EVAN_WEBCAM_PARAMS

namespace AR
{
/*
  Camera parameters
  --------------------
  These are obtained from camera calibration and are unique to and constant for
  every camera. Change the macro above to use a different set of camera parameters.
*/

// clang-format off
#ifndef USE_CAMERA_PARAMS
#error "USE_CAMERA_PARAMS macro is not defined"
#elif USE_CAMERA_PARAMS == EVAN_LAPTOP_CAMERA_PARAMS
	// Parameters for Evan's laptop camera:
	const double CAMERA_PARAMS_ARRAY[9] = {5.6534586568739849e+02, 0., 3.5588060437029822e+02,
										   0.,5.6534586568739849e+02, 2.9541081641775287e+02,
										   0., 0., 1.};
	const double DISTORTION_PARAMS_ARRAY[5]	= {3.4425270669197583e-01, -4.5627505509852968e+00,
											   1.4729564760154447e-04, -1.2416402052553264e-02,
											   1.5720190712074883e+01};
#elif USE_CAMERA_PARAMS == EVAN_WEBCAM_PARAMS

	//Parameters for Evan's webcam:
	const double CAMERA_PARAMS_ARRAY[9]	= {5.5929841506515140e+02, 0., 3.3632998221165434e+02,
										   0.,5.5929841506515140e+02, 2.4560280586167062e+02,
										   0., 0., 1.};
	const double DISTORTION_PARAMS_ARRAY[5]	= {2.2319725483552814e-02, -2.3138101190867111e-01,
											   3.6220766734074462e-03, 3.8852893952725500e-03,
											   5.4773015987500950e-01};
#else
#error "Unknown value for USE_CAMERA_PARAMS"
#endif
// clang-format on

// Matrices constructed from Camera and Distortion Parameters
/** OpenCV matrix containing the intrinsic camera parameters. */
const cv::Mat CAMERA_PARAMS(3, 3, CV_64FC1, *CAMERA_PARAMS_ARRAY);
/** OpenCV matrix containing the distortion parameters. */
const cv::Mat DISTORTION_PARAMS(5, 1, CV_64FC1, *DISTORTION_PARAMS_ARRAY);

/**
   Struct type representing a corner of a tag.
   Includes internal angle and the coordinates as an OpenCV Point.
 */
struct Corner
{
	double angle;
	cv::Point point;
};

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
	std::vector<Corner> corners;
	/** Stores the orientation Euler angles */
	cv::Vec3d orientation;
	cv::Vec3d rvec;
	cv::Vec3d tvec;
	/** Calculates the orientation Euler angles */
	void calcOrientation();

  public:
	/**
	   Constructor for a Tag. Takes four OpenCV points which are the coordinates of the
	   corners of the Tag in the image. Note the order of points; this will be checked and
	   an AR::InvalidCornerException will be thrown if ordering is incorrect.
	 */
	Tag(cv::Point top_left, cv::Point top_right, cv::Point bottom_right,
	    cv::Point bottom_left);
	/** Gets the coordinates of the center of the tag. */
	cv::Point getCenter() const;
	/**
	    Gets the corners of the tag. Note that this is a const operation and a copy of the
	    corners will be returned.
	*/
	std::vector<Corner> getCorners() const;
	/** Gets the pitch (rotation about x axis) of the tag */
	float getPitch() const;
	/** Gets the yaw (rotation about y axis) of the tag */
	float getYaw() const;
	/** Gets the roll (rotation about z axis) of the tag */
	float getRoll() const;
	/** Gets the distance from the camera to the tag. CURRENTLY UNIMPLEMENTED. */
	float getDistance();
	cv::Vec3d getRVec() const;
	cv::Vec3d getTVec() const;
};
} // namespace AR
