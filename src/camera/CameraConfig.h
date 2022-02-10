#pragma once

#include "CameraParams.h"

#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace cam {

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
	 * @brief If specified, gives the file to which the camera should be streamed.
	 */
	std::optional<std::string> filename;
	/**
	 * @brief If specified, gives the id of the camera.
	 */
	std::optional<int> cameraID;
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
