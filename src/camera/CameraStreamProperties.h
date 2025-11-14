#pragma once

#include "CameraConfig.h"
#include "CameraParams.h"

#include <string>

namespace cam {

struct CameraStreamProperties {
	int cameraId;
	std::string format;
	int width;
	int height;
	int framerate;
};

CameraStreamProperties readCameraStreamProperties(const std::string& configPath);

} // namespace cam
