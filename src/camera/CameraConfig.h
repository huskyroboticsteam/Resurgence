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

struct CameraConfig {
	std::string name;
	std::optional<CameraParams> intrinsicParams;
	std::optional<cv::Mat> extrinsicParams;
	std::optional<std::string> filename;
	std::optional<int> cameraID;
	std::optional<std::string> description;
};

CameraConfig readConfigFromFile(std::string filename);

} // namespace cam
