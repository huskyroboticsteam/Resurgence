#pragma once

#include "../simulator/utils.h"

namespace AR {
bool initializeLandmarkDetection();
bool isLandmarkDetectionInitialized();
points_t readLandmarks();
} // namespace AR
