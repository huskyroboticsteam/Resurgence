#include "CameraConfig.h"

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

invalid_camera_config::invalid_camera_config() : _msg("Invalid camera configuration") {}

invalid_camera_config::invalid_camera_config(const std::string& msg)
	: _msg("Invalid camera configuration: " + msg) {}

const char* invalid_camera_config::what() const noexcept {
	return _msg.c_str();
}

CameraConfig readConfigFromFile(std::string filename) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::invalid_argument("Configuration file " + filename + " does not exist");
	}

	CameraConfig cfg{};

	// read intrinsic parameters
	if (!fs[KEY_INTRINSIC_PARAMS].empty()) {
		CameraParams intrinsics;
		fs[KEY_INTRINSIC_PARAMS] >> intrinsics;
		cfg.intrinsicParams = intrinsics;
	}

	// read extrinsic parameters
	if (!fs[KEY_EXTRINSIC_PARAMS].empty()) {
		cv::Mat extrinsics;
		fs[KEY_EXTRINSIC_PARAMS] >> extrinsics;
		cfg.extrinsicParams = extrinsics;
	}

	// read name
	if (fs[KEY_NAME].empty()) {
		throw invalid_camera_config(KEY_NAME + " must be present");
	}
	cfg.name = static_cast<std::string>(fs[KEY_NAME]);

	// read description
	if (!fs[KEY_DESCRIPTION].empty()) {
		cfg.description = static_cast<std::string>(fs[KEY_DESCRIPTION]);
	}

	// read filename or camera ID, and open camera.
	if (!fs[KEY_FILENAME].empty()) {
		cfg.filename = static_cast<std::string>(fs[KEY_FILENAME]);
	}

	if (!fs[KEY_CAMERA_ID].empty()) {
		cfg.cameraID = static_cast<int>(fs[KEY_CAMERA_ID]);
	}

	if (!cfg.filename && !cfg.cameraID) {
		throw invalid_camera_config("One of " + KEY_FILENAME + " or " + KEY_CAMERA_ID +
									" must be present");
	}

	return cfg;
}

} // namespace cam
