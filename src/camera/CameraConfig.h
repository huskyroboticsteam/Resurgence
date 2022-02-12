#pragma once

#include "CameraParams.h"

#include <optional>
#include <string>
#include <variant>

#include <opencv2/core.hpp>

namespace cam {

/**@{*/
/**
   Config file key for camera filename.
 */
const std::string KEY_FILENAME = "filename";
/**
   Config file key for camera id.
 */
const std::string KEY_CAMERA_ID = "camera_id";
/**
   Config file key for intrinsic parameters.
 */
const std::string KEY_INTRINSIC_PARAMS = "intrinsic_params";
/**
   Config file key for extrinsic parameters.
 */
const std::string KEY_EXTRINSIC_PARAMS = "extrinsic_params";
/**
   Config file key for calibration information.
 */
const std::string KEY_CALIB_INFO = "calib_info";
/**
   Config file key for camera name.
 */
const std::string KEY_NAME = "name";
/**
   Config file key for camera description.
 */
const std::string KEY_DESCRIPTION = "description";
/**@}*/

/**
Exception for errors in the camera configuration.
*/
class invalid_camera_config : public std::exception {
public:
	/**
	   Constructs an invalid_camera_config exception with the default message "Invalid camera
	   configuration".
	 */
	invalid_camera_config();
	/**
	   Constructs an invalid_camera_config exception with the given message appended to
	   "Invalid camera configuration:".
	   @param msg The message to use for the exception.
	 */
	invalid_camera_config(const std::string& msg);
	/**
	   Returns the exception message as a C string.
	 */
	virtual const char* what() const noexcept;

private:
	std::string _msg;
};

/**
 * @brief A struct that represents the information outlined in @ref cameraparams.
 */
struct CameraConfig {
	/**
	 * @brief The name of the camera.
	 */
	std::string name;

	/**
	 * @brief If specified, gives the intrinsic parameter matrix.
	 */
	std::optional<CameraParams> intrinsicParams;

	/**
	 * @brief If specified, gives the extrinsic parameter matrix.
	 */
	std::optional<cv::Mat> extrinsicParams;

	/**
	 * @brief Either the camera ID or the filename to stream from.
	 *
	 * The camera ID specifies the ID of the camera device to use.
	 * If the filename is specified, this camera will stream from a video file.
	 */
	std::variant<std::string, int> filenameOrID;

	/**
	 * @brief If specified, gives the text description of the camera.
	 */
	std::optional<std::string> description;
};

/**
 * @brief Read the camera config from the specified file.
 *
 * @param filename The path to the configuration file to open and read. Configuration file
 * should be formatted as described in @ref cameraconfig.
 *
 * @return CameraConfig The parsed camera config object.
 *
 * @throws invalid_camera_config If the configuration is invalid for any reason.
 *
 * @throws invalid_argument If the file does not exist.
 */
CameraConfig readConfigFromFile(const std::string& filename);

} // namespace cam
