#pragma once

#include "../world_interface/data.h"

namespace AR {
bool initializeLandmarkDetection();
bool isLandmarkDetectionInitialized();
landmarks_t readLandmarks();
} // namespace AR
