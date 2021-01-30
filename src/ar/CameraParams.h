#pragma once

#include <opencv2/core.hpp>

namespace AR
{

/**
  Camera parameters (also called "intrinsic parameters") are obtained from camera calibration
  and are unique to and constant for every camera. They define how the camera projects a 3D
  space onto a 2D image plane.

  Each set of CameraParams contains a camera matrix which defines how points in 3D space are
  linearly projected to the 2D image plane of the camera, and a set of distortion coefficients
  which can be used to correct for radial distortion in the image.

  Note that clients of the AR tag detection system should not need to construct CameraParams
  objects; see getCameraParams() below.
*/
class CameraParams
{
private:
	cv::Mat _camera_params;
	cv::Mat _dist_coeff;

public:
	CameraParams(cv::Mat camera_params, cv::Mat dist_coeff);
	/**
	   Gets the camera matrix associated with this set of camera parameters.
	 */
	cv::Mat getCameraParams() const;
	/**
	   Gets the distortion coefficients (as a column vector) associated with this set of camera
	   parameters.
	 */
	cv::Mat getDistCoeff() const;
};

/**
   The set of all the camera parameters that are defined so far. Note that these are just
   constants to identify a set of camera parameters; you should call getCameraParams() with one
   of these values as an argument to get an actual CameraParams object that you can use.

   Each member should have a comment describing which camera they are for, what the image size
   is, and what the unit scale is. It is important to use the correct image size and unt
   scale. 

   Note that some of these are old or were just for testing and are probably subject to be
   removed at some point in the future.
 */
enum Params
{

	/**
	   Params for Winston's webcam, 640x480, scale in meters
	 */
	WINSTON_WEBCAM_480,
	/**
	   Params for Evan's webcam, 1920x1080, scale in meters
	 */
	WEBCAM_1080,
	/**
	   Params for Evan's webcam, 1280x720, scale in meters
	 */
	WEBCAM_720,
	/**
	   Params for Evan's integrated laptop camera, 640x480, scale in millimeters
	 */
	LAPTOP,
	/**
	   Params for Evan's webcam, 640x480, scale in millimeters
	 */
	WEBCAM,
	/**
	   Params for Evan's new webcam, 640x480, scale in meters
	 */
	EVAN_NEW_WEBCAM_480,
	/**
	   Params for the webcam on top of the robot, 640x480, scale in meters
	 */
	ROBOT_TOP_WEBCAM_480

};

/**
   Returns a CameraParams object corresponding to the given Params constant.
 */
CameraParams getCameraParams(Params params);

} // namespace AR
