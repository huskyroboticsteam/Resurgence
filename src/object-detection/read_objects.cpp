#include "read_objects.h"

#include "../Constants.h"
#include "../Globals.h"
#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
#include "../world_interface/world_interface.h"
#include "ObjectDetector.h"

#include <atomic>
#include <filesystem>
#include <loguru.hpp>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

using namespace robot::types;

namespace ObjDet {

// Model and class configuration
const std::vector<std::string> DEFAULT_CLASSES = {
    "no object",
    "orange mallet or hammer",
    "water bottle"
};

// Find model file by checking multiple possible locations
std::string findModelPath() {
    // 1. Check environment variable
    const char* env_path = std::getenv("OWLVIT_MODEL_PATH");
    if (env_path && std::filesystem::exists(env_path)) {
        return env_path;
    }
    
    // 2. Check current directory (for backward compatibility)
    if (std::filesystem::exists("owlvit-cpp.pt")) {
        return "owlvit-cpp.pt";
    }
    
    // 3. Check in ../src/object-detection/ (when running from build/)
    if (std::filesystem::exists("../src/object-detection/owlvit-cpp.pt")) {
        return "../src/object-detection/owlvit-cpp.pt";
    }
    
    // 4. Check in src/object-detection/ (when running from project root)
    if (std::filesystem::exists("src/object-detection/owlvit-cpp.pt")) {
        return "src/object-detection/owlvit-cpp.pt";
    }
    
    throw std::runtime_error("Could not find owlvit-cpp.pt model file. "
                           "Please set OWLVIT_MODEL_PATH environment variable or place the model in the current directory.");
}

// Global detector instance
ObjectDetector obj_detector;

// Thread synchronization
std::atomic<bool> fresh_data(false);
std::mutex detection_lock;
std::vector<DetectionResult> current_detections;
std::thread detection_thread;
bool initialized = false;

void detectObjectsLoop() {
    loguru::set_thread_name("ObjectDetection");
    cv::Mat frame;
    uint32_t last_frame_no = 0;
    bool was_enabled = false;
    
    while (true) {
        // Check if detection is enabled (using global flag)
        if (!Globals::objectDetectionEnabled) {
            if (was_enabled) {
                LOG_F(INFO, "Object detection loop: STOPPED (disabled)");
                was_enabled = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        
        if (!was_enabled) {
            LOG_F(INFO, "Object detection loop: STARTED (enabled)");
            was_enabled = true;
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
            LOG_F(INFO, "Object detection: %ld object(s) detected", detections.size());
            
            // Update shared results
            detection_lock.lock();
            current_detections = detections;
            fresh_data = true;
            detection_lock.unlock();
            
            // Log detected objects
            for (const auto& det : detections) {
                LOG_F(INFO, "  - %s (confidence: %.3f) at [%d, %d, %dx%d]", 
                      det.class_name.c_str(), 
                      det.confidence,
                      det.bounding_box.x, 
                      det.bounding_box.y,
                      det.bounding_box.width,
                      det.bounding_box.height);
            }
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
        std::string model_path = findModelPath();
        LOG_F(INFO, "Using OWL-ViT model from: %s", model_path.c_str());
        
        // Initialize detector
        obj_detector = ObjectDetector(
            DEFAULT_CLASSES,
            model_path,
            0.6f,  // Default confidence threshold
            config.intrinsicParams.value()
        );
        
        // Start detection thread
        detection_thread = std::thread(&detectObjectsLoop);
        
        LOG_F(INFO, "Object detection initialized successfully");
        
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

} // namespace ObjDet
