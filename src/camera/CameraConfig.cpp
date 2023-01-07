#include "CameraConfig.h"
#include <opencv2/core.hpp>
#include <vector>
namespace cam {

invalid_camera_config::invalid_camera_config() : _msg("Invalid camera configuration") {}

invalid_camera_config::invalid_camera_config(const std::string& msg)
	: _msg("Invalid camera configuration: " + msg) {}

const char* invalid_camera_config::what() const noexcept {
	return _msg.c_str();
}

CameraConfig readConfigFromFile(const std::string& filename) {
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::invalid_argument("Configuration file " + filename + " does not exist");
	}

	CameraConfig cfg{};

	// read intrinsic parameters
	if (!fs[KEY_INTRINSIC_PARAMS].empty()) {
		CameraParams intrinsics;
		std::vector<double> intrinsic_list1d;
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
		cfg.filenameOrID = static_cast<std::string>(fs[KEY_FILENAME]);
	} else if (!fs[KEY_CAMERA_ID].empty()) {
		cfg.filenameOrID = static_cast<int>(fs[KEY_CAMERA_ID]);
	} else {
		throw invalid_camera_config("One of " + KEY_FILENAME + " or " + KEY_CAMERA_ID +
									" must be present");
	}

	return cfg;
}

} // namespace cam
