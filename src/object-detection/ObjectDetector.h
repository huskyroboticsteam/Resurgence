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
 * @brief Object detector using OWL-ViT model for open-vocabulary object detection.
 * 
 * This class encapsulates a PyTorch-based object detection model that can detect
 * objects based on text descriptions. It provides functionality similar to AR::Detector
 * with enable/disable capability.
 */
class ObjectDetector {
private:
    torch::jit::script::Module model_;
    std::vector<std::string> class_names_;
    cam::CameraParams camera_params_;
    cv::Mat map1_, map2_;  // Undistortion maps
    
    // Pre-computed tokens for text prompts
    torch::Tensor input_ids_;
    torch::Tensor attention_mask_;
    
    float confidence_threshold_;
    bool enabled_;
    torch::Device device_;
    
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

public:
    /**
     * @brief Default constructor. Creates an empty detector.
     */
    ObjectDetector();
    
    /**
     * @brief Construct an ObjectDetector with specified parameters.
     * 
     * @param class_names List of class names to detect (first should be "no object")
     * @param model_path Path to the traced OWL-ViT model (.pt file)
     * @param confidence_threshold Minimum confidence threshold for detections
     * @param camera_params Camera parameters for undistortion (optional)
     */
    ObjectDetector(const std::vector<std::string>& class_names,
                   const std::string& model_path,
                   float confidence_threshold = 0.6f,
                   const cam::CameraParams& camera_params = cam::CameraParams());
    
    /**
     * @brief Detect objects in an image.
     * 
     * @param image Input image
     * @param undistort Whether to apply undistortion before detection
     * @return Vector of detected objects
     */
    std::vector<DetectionResult> detect(const cv::Mat& image, bool undistort = false);
    
    /**
     * @brief Draw bounding boxes and labels on image.
     * 
     * @param image Image to draw on (modified in-place)
     * @param results Detection results to visualize
     */
    void drawDetections(cv::Mat& image, const std::vector<DetectionResult>& results) const;
    
    /**
     * @brief Enable or disable the detector.
     * 
     * When disabled, detect() will return empty results immediately.
     * 
     * @param enabled True to enable, false to disable
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Check if detector is enabled.
     * 
     * @return True if enabled, false otherwise
     */
    bool isEnabled() const;
    
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
