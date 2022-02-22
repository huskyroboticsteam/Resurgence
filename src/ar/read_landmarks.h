#pragma once

#include "../world_interface/data.h"

namespace AR {
bool initializeLandmarkDetection();
bool isLandmarkDetectionInitialized();
robot::types::landmarks_t readLandmarks();
} // namespace AR
