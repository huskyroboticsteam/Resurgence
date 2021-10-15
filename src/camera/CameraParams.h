#pragma once

#include <opencv2/core.hpp>

namespace cam {

/**
   @addtogroup camera
   @{
 */

/**
	@name Configuration File Keys

	The following are constants for the keys used in the camera configuration files. See @ref
	cameraconfig for more details.
 */
/**@{*/
/**
   Config file key for image width.
 */
const std::string KEY_IMAGE_WIDTH = "image_width";
/**
   Config file key for image height.
 */
const std::string KEY_IMAGE_HEIGHT = "image_height";
/**
   Config file key for the camera matrix.
 */
const std::string KEY_CAMERA_MATRIX = "camera_matrix";
/**
   Config file key for the distortion coefficients.
 */
const std::string KEY_DIST_COEFFS = "distortion_coefficients";
/**@}*/

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
class CameraParams {
private:
	cv::Mat _camera_matrix;
	cv::Mat _dist_coeff;
	cv::Size _image_size;
	void init(const cv::Mat& camera_matrix, const cv::Mat& dist_coeff, cv::Size image_size);

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
	CameraParams(const cv::Mat& camera_matrix, const cv::Mat& dist_coeff,
				 cv::Size image_size = cv::Size(640, 480));
	/**
	   @brief Copy constructor.

	   Constructs a set of camera parameters by copying another. Underlying data like the
	   camera matrix and distortion coefficients are actually copied, so the copy may be
	   modified without affecting the original.
	 */
	CameraParams(const CameraParams& other);
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
	void readFromFileNode(const cv::FileNode& file_node);
	/**
	   @brief Writes the data for this CameraParams object to the given cv::FileStorage
	   object.

	   Used for serialization - you should not need to call this method directly but should
	   instead use the << operator on a cv::FileStorage object.
	 */
	void writeToFileStorage(cv::FileStorage& file_storage) const;
};

/**
   @brief Reads a CameraParams object from the given cv::FileNode object.

   Used for serialization - you should not need to call this method directly but should
   instead use the >> operator on a cv::FileNode object.
*/
void read(const cv::FileNode& node, CameraParams& params,
		  const CameraParams& default_value = CameraParams());
/**
   @brief Writes the given CameraParams object to the given cv::FileStorage object.

   Used for serialization - you should not need to call this method directly but should
   instead use the << operator on a cv::FileStorage object.
*/
void write(cv::FileStorage& fs, const std::string& name, const CameraParams& params);

/** @} */

} // namespace cam
