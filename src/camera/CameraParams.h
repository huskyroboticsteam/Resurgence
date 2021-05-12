#pragma once

#include <opencv2/core.hpp>

namespace cam
{

/**
   @addtogroup camera
   @{
 */

const std::string KEY_IMAGE_WIDTH = "image_width";
const std::string KEY_IMAGE_HEIGHT = "image_height";
const std::string KEY_CAMERA_MATRIX = "camera_matrix";
const std::string KEY_DIST_COEFFS = "distortion_coefficients";

/**
   @brief Represents a set of intrinsic camera parameters.

   Camera parameters (also called "intrinsic parameters") are obtained from
   camera calibration and are unique to and constant for every camera. They
   define how the camera projects a 3D space onto a 2D image plane.

   Each set of CameraParams contains a camera matrix which defines how points in
   3D space are linearly projected to the 2D image plane of the camera, and a
   set of distortion coefficients which can be used to correct for radial
   distortion in the image. The projection is linear which means that projected
   points are unique only up to scaling.

   @warning Note that camera parameters are different for different resolutions
   of the camera! It is often not enough to simply scale the image centers/focal
   lengths for different resolutions because many cameras have different fields
   of view (and therefore different projections/distortion) at different
   resolutions. For this reason, a CameraParams object has an associated
   cv::Size which contains the image size the parameters are valid for.
*/
class CameraParams
{
private:
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeff;
	cv::Size _image_size;
	void init(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff, cv::Size image_size);

public:
	/**
	   Constructs a default or empty set of camera parameters.

	   @warning Empty sets of camera parameters are not suitable for actual use! Check to make
	   sure they are non-empty with empty().
	 */
	CameraParams();
	/**
	   Constructs a set of camera parameters with the given camera matrix, distortion
	   coefficients, and image size.

	   @param camera_matrix A 3x3 matrix defining the projection of the camera.

	   @param dist_coeff A set of distortion coefficients defining the lens distortion of the
	   camera. Must be a 1xN or Nx1 matrix where N is the number of coefficients. N must be 4,
	   5, 8, 12, or 14.

	   @param image_size The size of the image the parameters are calibrated for. Defaults to
	   640x480 as this is the default resolution of many webcams.
	 */
	CameraParams(const cv::Mat &camera_matrix, const cv::Mat &dist_coeff,
				 cv::Size image_size = cv::Size(640, 480));
	/**
	   @brief Copy constructor.

	   Constructs a set of camera parameters by copying another. Underlying data like the
	   camera matrix and distortion coefficients are actually copied, so the copy may be
	   modified without affecting the original.
	 */
	CameraParams(const CameraParams &other);
	bool empty() const;
	/**
	   Gets the camera matrix associated with this set of camera parameters.
	 */
	cv::Mat getCameraMatrix() const;
	/**
	   Gets the distortion coefficients (as a column vector) associated with
	   this set of camera parameters.
	 */
	cv::Mat getDistCoeff() const;
	/**
	   Gets the image size this set of intrinsic parameters was calibrated for.
	 */
	cv::Size getImageSize() const;

	/**
	   @brief Reads the data for this CameraParams object from the given cv::FileNode object.

	   Used for serialization - you should not need to call this method directly but should
	   instead use the >> operator on a cv::FileNode object.
	 */
	void readFromFileNode(const cv::FileNode &file_node);
	/**
	   @brief Writes the data for this CameraParams object to the given cv::FileStorage
	   object.

	   Used for serialization - you should not need to call this method directly but should
	   instead use the << operator on a cv::FileStorage object.
	 */
	void writeToFileStorage(cv::FileStorage &file_storage) const;
};

/**
   @brief Reads a CameraParams object from the given cv::FileNode object.

   Used for serialization - you should not need to call this method directly but should
   instead use the >> operator on a cv::FileNode object.
*/
void read(const cv::FileNode &node, CameraParams &params,
		  const CameraParams &default_value = CameraParams());
/**
   @brief Writes the given CameraParams object to the given cv::FileStorage object.

   Used for serialization - you should not need to call this method directly but should
   instead use the << operator on a cv::FileStorage object.
*/
void write(cv::FileStorage &fs, const std::string &name, const CameraParams &params);

/**
   The set of all the camera parameters that are defined so far. Note that these are just
   constants to identify a set of camera parameters; you should call getCameraParams() with one
   of these values as an argument to get an actual CameraParams object that you can use.

   Each member should have a comment describing which camera they are for and what the image
   size is. It is important to use the correct image size.

   Note that some of these are old or were just for testing and are probably subject to be
   removed at some point in the future.

   TODO: This needs to be reworked as we flesh out the camera system.
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

} // namespace cam
