#include "read_objects.h"

#include "../Constants.h"
#include "../Globals.h"
#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
#include "../world_interface/world_interface.h"
#include "ObjectDetector.h"
#include "ModelDownloader.h"

#include <atomic>
#include <filesystem>
#include <loguru.hpp>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

using namespace robot::types;

namespace ObjDet {

// Global detector instance
ObjectDetector obj_detector;

// Thread synchronization
std::atomic<bool> fresh_data(false);
std::mutex detection_lock;
std::vector<DetectionResult> current_detections;
std::thread detection_thread;
bool initialized = false;

// Track last active task for logging
DetectionTask last_logged_task = DetectionTask::NONE;

void detectObjectsLoop() {
    loguru::set_thread_name("ObjectDetection");
    cv::Mat frame;
    uint32_t last_frame_no = 0;
    
    while (true) {
        // Check current active task
        DetectionTask current_task = obj_detector.getActiveTask();
        
        // Log task changes
        if (current_task != last_logged_task) {
            if (current_task == DetectionTask::NONE) {
                LOG_F(INFO, "Object detection: DISABLED");
            } else {
                LOG_F(INFO, "Object detection: Switched to task '%s'", 
                      ObjectDetector::getTaskName(current_task).c_str());
            }
            last_logged_task = current_task;
        }
        
        // Skip if no task is active
        if (current_task == DetectionTask::NONE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        // Get new camera frame
        if (robot::hasNewCameraFrame(Constants::MAST_CAMERA_ID, last_frame_no)) {
            auto camData = robot::readCamera(Constants::MAST_CAMERA_ID);
            if (!camData)
                continue;
            
            auto camFrame = camData.getData();
            frame = camFrame.first;
            last_frame_no = camFrame.second;
            
            // Run object detection
            std::vector<DetectionResult> detections = obj_detector.detect(frame);
            
            if (!detections.empty()) {
                LOG_F(INFO, "Object detection [%s]: %ld object(s) detected", 
                      ObjectDetector::getTaskName(current_task).c_str(),
                      detections.size());
                
                // Log detected objects
                for (const auto& det : detections) {
                    LOG_F(INFO, "  - %s (confidence: %.3f, distance: %.2fm) at [%d, %d, %dx%d]", 
                          det.class_name.c_str(), 
                          det.confidence,
                          det.actual_distance_meters,
                          det.bounding_box.x, 
                          det.bounding_box.y,
                          det.bounding_box.width,
                          det.bounding_box.height);
                }
            }
            
            // Update shared results
            detection_lock.lock();
            current_detections = detections;
            fresh_data = true;
            detection_lock.unlock();
        }
        
        // Small sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool initializeObjectDetection() {
    // Load camera intrinsic parameters from config file
    try {
        auto config = cam::readConfigFromFile(
            Constants::CAMERA_CONFIG_PATHS.at(Constants::MAST_CAMERA_ID)
        );
        
        if (!config.intrinsicParams || config.intrinsicParams->empty()) {
            LOG_F(ERROR, "Camera configuration does not have intrinsic parameters! "
                         "Object detection cannot be performed.");
            return false;
        }
        
        // Find model file
        std::string model_path = findOrDownloadModel();
        LOG_F(INFO, "Using OWL-ViT model from: %s", model_path.c_str());
        
        // Initialize detector with task-based configuration
        obj_detector = ObjectDetector(
            model_path,
            0.6f,  // Default confidence threshold
            config.intrinsicParams.value()
        );
        
        // Start detection thread
        detection_thread = std::thread(&detectObjectsLoop);
        
        LOG_F(INFO, "Object detection initialized successfully");
        LOG_F(INFO, "Available tasks:");
        LOG_F(INFO, "  Key '1': Orange Hammer");
        LOG_F(INFO, "  Key '2': Rock Pick");
        LOG_F(INFO, "  Key '3': Water Bottle");
        LOG_F(INFO, "  Key '4': All Objects");
        
        if (!config.extrinsicParams || config.extrinsicParams->empty()) {
            LOG_F(WARNING, "Camera configuration does not have extrinsic parameters! "
                          "Coordinates returned for objects will be relative to camera");
        }
        
    } catch (const std::exception& e) {
        LOG_F(ERROR, "Failed to initialize object detection: %s", e.what());
        return false;
    }
    
    initialized = true;
    return true;
}

bool isObjectDetectionInitialized() {
    return initialized;
}

std::vector<DetectionResult> readDetectedObjects() {
    if (!isObjectDetectionInitialized()) {
        return {};
    }
    
    if (fresh_data) {
        std::vector<DetectionResult> output;
        detection_lock.lock();
        output = current_detections;
        fresh_data = false;
        detection_lock.unlock();
        return output;
    }
    
    return {};
}

void setActiveTask(DetectionTask task) {
    if (!isObjectDetectionInitialized()) {
        return;
    }
    obj_detector.setActiveTask(task);
}

DetectionTask getActiveTask() {
    if (!isObjectDetectionInitialized()) {
        return DetectionTask::NONE;
    }
    return obj_detector.getActiveTask();
}

void toggleTask(DetectionTask task) {
    if (!isObjectDetectionInitialized()) {
        return;
    }
    obj_detector.toggleTask(task);
}

bool isDetectionEnabled() {
    if (!isObjectDetectionInitialized()) {
        return false;
    }
    return obj_detector.isEnabled();
}

ObjectDetector& getDetector() {
    return obj_detector;
}

} // namespace ObjDet
