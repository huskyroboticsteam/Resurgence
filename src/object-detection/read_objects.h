#pragma once

#include "DetectionResult.h"
#include <vector>

namespace ObjDet {

/**
 * @brief Initialize object detection system.
 * 
 * Loads the model and sets up the detector with camera parameters.
 * 
 * @return true if initialization succeeded, false otherwise
 */
bool initializeObjectDetection();

/**
 * @brief Check if object detection is initialized.
 * 
 * @return true if initialized, false otherwise
 */
bool isObjectDetectionInitialized();

/**
 * @brief Read the latest detected objects.
 * 
 * Returns cached results from the detection loop. If no fresh data is available,
 * returns empty vector. Detection is controlled via Globals::objectDetectionEnabled.
 * 
 * @return Vector of detected objects
 */
std::vector<DetectionResult> readDetectedObjects();

} // namespace ObjDet
