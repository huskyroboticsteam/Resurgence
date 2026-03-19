#pragma once

#include "DetectionResult.h"
#include "../camera/CameraParams.h"

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <torch/script.h>
#include <torch/torch.h>

namespace ObjDet {

/**
 * @brief Available detection tasks (mutually exclusive).
 * 
 * Only one task can be active at a time.
 * Each task has its own model, class names, and language prompts.
 */
enum class DetectionTask {
    NONE,           ///< All detection disabled
    ORANGE_HAMMER,  ///< Detect orange/construction mallet (Key '1')
    ROCK_PICK,      ///< Detect rock pick hammer (Key '2')
    WATER_BOTTLE,   ///< Detect water bottle (Key '3')
    ALL             ///< Detect all objects (Key '4')
};

/**
 * @brief Configuration for a detection task.
 *
 * Each task specifies which class indices (from the global class list) to accept.
 * The fine-tuned model always runs with all classes; tasks filter the results.
 */
struct TaskConfig {
    std::vector<int> accepted_class_indices;     ///< Which class indices this task accepts
};

/**
 * @brief Object detector using OWL-ViT model for open-vocabulary object detection.
 * 
 * This class encapsulates a PyTorch-based object detection model that can detect
 * objects based on text descriptions. It supports multiple detection tasks with
 * different models and prompts, where only one task can be active at a time.
 */
class ObjectDetector {
private:
    torch::jit::script::Module model_;
    // Global class list (matches fine-tuned model: no object, orange mallet, rock pick hammer, water bottle)
    std::vector<std::string> class_names_;
    cam::CameraParams camera_params_;
    cv::Mat map1_, map2_;  // Undistortion maps

    float confidence_threshold_;

    // Task management (device_ must be before active_task_ for initialization order)
    DetectionTask active_task_;
    torch::Device device_;
    std::map<DetectionTask, TaskConfig> task_configs_;

    // Real-world object dimensions (in meters) for distance estimation
    std::map<std::string, float> object_heights_;
    std::map<std::string, float> object_widths_;
    
    /**
     * @brief Preprocess image for model input.
     * 
     * Resizes to 768x768, converts BGR to RGB, normalizes with mean=[0.5, 0.5, 0.5]
     * and std=[0.5, 0.5, 0.5].
     * 
     * @param image Input image in BGR format
     * @return Preprocessed tensor ready for model input
     */
    torch::Tensor preprocess(const cv::Mat& image);
    
    /**
     * @brief Run OWL-ViT model inference.
     * 
     * @param image Input image
     * @return Vector containing [logits, predicted_boxes]
     */
    std::vector<torch::Tensor> runModel(const cv::Mat& image);
    
    /**
     * @brief Initialize undistortion maps from camera parameters.
     */
    void initUndistortMaps();
    
    /**
     * @brief Apply Non-Maximum Suppression to remove overlapping detections.
     * 
     * @param detections Input detection results
     * @param nms_threshold IoU threshold for suppression (default: 0.5)
     * @return Filtered detection results
     */
    std::vector<DetectionResult> applyNMS(const std::vector<DetectionResult>& detections, 
                                          float nms_threshold = 0.5f);
    
    /**
     * @brief Calculate actual distance using pinhole camera model.
     * 
     * Uses both height and width for improved accuracy:
     * - distance_h = (real_height × focal_length_y) / pixel_height
     * - distance_w = (real_width × focal_length_x) / pixel_width
     * - final_distance = average of both
     * 
     * @param bbox Bounding box of detected object
     * @param class_name Name of the detected object class
     * @return Estimated distance in meters (-1.0 if calculation not possible)
     */
    float calculateActualDistance(const cv::Rect& bbox, const std::string& class_name) const;
    
    /**
     * @brief Initialize default object dimensions.
     * 
     * Sets typical heights for known objects (hammer, water bottle, etc.)
     */
    void initializeObjectDimensions();
    
    /**
     * @brief Initialize task configurations.
     * 
     * Sets up model paths, class names, and tokenized prompts for each task.
     */
    void initializeTaskConfigs();
    
    /**
     * @brief Load model for a specific task.
     * 
     * @param task The task to load the model for
     */
    void loadModelForTask(DetectionTask task);

public:
    /**
     * @brief Default constructor. Creates an empty detector.
     */
    ObjectDetector();
    
    /**
     * @brief Construct an ObjectDetector with specified parameters.
     * 
     * @param default_model_path Default path to the traced OWL-ViT model (.pt file)
     * @param confidence_threshold Minimum confidence threshold for detections
     * @param camera_params Camera parameters for undistortion (optional)
     */
    ObjectDetector(const std::string& default_model_path,
                   float confidence_threshold = 0.6f,
                   const cam::CameraParams& camera_params = cam::CameraParams());
    
    /**
     * @brief Detect objects in an image using the active task.
     * 
     * @param image Input image
     * @param undistort Whether to apply undistortion before detection
     * @param estimate_distance Whether to calculate actual distance using camera model
     * @return Vector of detected objects with distance estimates in meters
     */
    std::vector<DetectionResult> detect(const cv::Mat& image, bool undistort = false, bool estimate_distance = true);
    
    /**
     * @brief Draw bounding boxes and labels on image.
     * 
     * @param image Image to draw on (modified in-place)
     * @param results Detection results to visualize
     */
    void drawDetections(cv::Mat& image, const std::vector<DetectionResult>& results) const;
    
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
     * @brief Get the currently active task.
     * 
     * @return The active detection task
     */
    DetectionTask getActiveTask() const;
    
    /**
     * @brief Toggle a specific task on/off.
     * 
     * If the task is currently active, it will be disabled (set to NONE).
     * If another task or NONE is active, the specified task will be activated.
     * 
     * @param task The task to toggle
     */
    void toggleTask(DetectionTask task);
    
    /**
     * @brief Check if any detection task is active.
     * 
     * @return True if a task is active, false if NONE
     */
    bool isEnabled() const;
    
    /**
     * @brief Get the name of a detection task.
     * 
     * @param task The task to get the name for
     * @return Human-readable task name
     */
    static std::string getTaskName(DetectionTask task);
    
    /**
     * @brief Calculate distance from depth image for detected objects.
     * 
     * Uses median depth from center region of bounding box for robustness.
     * This is the preferred method when RealSense depth data is available.
     * 
     * @param depth_frame 16-bit depth image (aligned to color)
     * @param depth_scale Depth scale factor (meters per unit, e.g., 0.001 for mm)
     * @param bbox Bounding box of the detected object
     * @return Distance in meters, or -1.0 if invalid
     */
    static float getDistanceFromDepth(const cv::Mat& depth_frame, float depth_scale,
                                       const cv::Rect& bbox);
    
    /**
     * @brief Detect objects and compute distances using depth image.
     * 
     * This method performs detection on the color image and uses the aligned
     * depth image to compute actual distances, replacing the pinhole model
     * estimation with direct depth measurement.
     * 
     * @param color_image Input color image (BGR)
     * @param depth_image Input depth image (16-bit, aligned to color)
     * @param depth_scale Depth scale factor (meters per unit)
     * @param undistort Whether to undistort the color image before detection
     * @return Vector of detected objects with depth-based distances
     */
    std::vector<DetectionResult> detectWithDepth(const cv::Mat& color_image,
                                                  const cv::Mat& depth_image,
                                                  float depth_scale,
                                                  bool undistort = false);
    
    /**
     * @brief Set confidence threshold for filtering detections.
     * 
     * @param threshold Minimum confidence value (0.0 to 1.0)
     */
    void setConfidenceThreshold(float threshold);
    
    /**
     * @brief Get current confidence threshold.
     * 
     * @return Current threshold value
     */
    float getConfidenceThreshold() const;
    
    /**
     * @brief Check if detector is properly initialized.
     * 
     * @return True if model is loaded and class names are set
     */
    bool empty() const;
};

} // namespace ObjDet
