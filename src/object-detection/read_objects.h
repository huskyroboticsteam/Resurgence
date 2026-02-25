#pragma once

#include "DetectionResult.h"
#include "ObjectDetector.h"  // For DetectionTask enum
#include <vector>

namespace ObjDet {

/**
 * @brief Initialize object detection system.
 * 
 * Loads the model and sets up the detector with camera parameters.
 * Detection starts in DISABLED state (task = NONE).
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
 * returns empty vector. Detection is controlled via setActiveTask() or toggleTask().
 * 
 * @return Vector of detected objects
 */
std::vector<DetectionResult> readDetectedObjects();

/**
 * @brief Set the active detection task.
 * 
 * Only one task can be active at a time. Setting a new task
 * automatically disables the previous one.
 * 
 * @param task The task to activate (use NONE to disable all)
 */
void setActiveTask(DetectionTask task);

/**
 * @brief Get the currently active detection task.
 * 
 * @return The active detection task
 */
DetectionTask getActiveTask();

/**
 * @brief Toggle a specific detection task on/off.
 * 
 * If the task is currently active, it will be disabled (set to NONE).
 * If another task or NONE is active, the specified task will be activated.
 * 
 * @param task The task to toggle
 */
void toggleTask(DetectionTask task);

/**
 * @brief Check if any detection task is currently active.
 * 
 * @return true if a task is active, false if NONE
 */
bool isDetectionEnabled();

/**
 * @brief Get the global ObjectDetector instance.
 * 
 * Use this for advanced configuration like setting confidence threshold.
 * 
 * @return Reference to the global ObjectDetector
 */
ObjectDetector& getDetector();

} // namespace ObjDet
