#include "CameraStreamProperties.h"

#include <opencv2/core.hpp>

#include <stdexcept>

namespace cam {

CameraStreamProperties readCameraStreamProperties(const std::string& configPath) {
	cv::FileStorage fs(configPath, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::invalid_argument("Configuration file " + configPath + " does not exist");
	}

	if (fs[KEY_CAMERA_ID].empty() || fs[KEY_FORMAT].empty() || fs[KEY_IMAGE_WIDTH].empty() ||
		fs[KEY_IMAGE_HEIGHT].empty() || fs[KEY_FRAMERATE].empty()) {
		throw invalid_camera_config(
			"Camera stream configuration missing required keys (camera_id, format, image_width, "
			"image_height, framerate)");
	}

	CameraStreamProperties props{};
	props.cameraId = static_cast<int>(fs[KEY_CAMERA_ID]);
	props.format = static_cast<std::string>(fs[KEY_FORMAT]);
	props.width = static_cast<int>(fs[KEY_IMAGE_WIDTH]);
	props.height = static_cast<int>(fs[KEY_IMAGE_HEIGHT]);
	props.framerate = static_cast<int>(fs[KEY_FRAMERATE]);

	return props;
}

} // namespace cam
