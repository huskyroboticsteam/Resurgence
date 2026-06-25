#include "ObjectDetector.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace ObjDet {

ObjectDetector::ObjectDetector() 
    : confidence_threshold_(0.5f), 
      active_task_(DetectionTask::NONE),
      device_(torch::kCPU) {
}

ObjectDetector::ObjectDetector(const std::string& default_model_path,
                               float confidence_threshold,
                               const cam::CameraParams& camera_params)
    : camera_params_(camera_params),
      confidence_threshold_(confidence_threshold),
      active_task_(DetectionTask::NONE),
      device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU) {

    // Disable JIT optimizations to avoid CUDA nvrtc compilation issues
    // This prevents "__ldg" undefined identifier errors on some CUDA versions
    torch::jit::setGraphExecutorOptimize(false);
    torch::jit::FusionStrategy strategy = {{torch::jit::FusionBehavior::STATIC, 0}};
    torch::jit::setFusionStrategy(strategy);

    // Initialize global class tokens and per-task configs
    initializeTaskConfigs();

    // Load the fine-tuned model
    try {
        model_ = torch::jit::load(default_model_path);
        model_.to(device_);
        model_.eval();
        std::cout << "ObjectDetector: Model loaded successfully from " << default_model_path << std::endl;
        std::cout << "ObjectDetector: Using device: " << (device_.is_cuda() ? "CUDA" : "CPU") << std::endl;
    } catch (const c10::Error& e) {
        std::cerr << "ObjectDetector: Error loading model from " << default_model_path << std::endl;
        std::cerr << e.what() << std::endl;
        throw std::runtime_error("Failed to load OWL-ViT model");
    }

    // Initialize undistortion maps if camera parameters are provided
    if (!camera_params_.empty()) {
        initUndistortMaps();
    }
}

void ObjectDetector::initializeTaskConfigs() {
    // ========== Global: Class names (text embeddings are baked into the model) ==========
    // Fine-tuned with 4 classes: ["no object", "orange mallet", "rock pick hammer", "water bottle"]
    class_names_ = {"no object", "orange mallet", "rock pick hammer", "water bottle"};

    // ========== Global: Object dimensions for distance estimation ==========
    object_heights_["orange mallet"] = 0.35f;
    object_widths_["orange mallet"] = 0.10f;
    object_heights_["rock pick hammer"] = 0.30f;
    object_widths_["rock pick hammer"] = 0.08f;
    object_heights_["water bottle"] = 0.20f;
    object_widths_["water bottle"] = 0.065f;

    // ========== Per-task: Which class indices to accept ==========
    task_configs_[DetectionTask::ORANGE_HAMMER] = {{1}};       // class 1: orange mallet
    task_configs_[DetectionTask::ROCK_PICK]     = {{2}};       // class 2: rock pick hammer
    task_configs_[DetectionTask::WATER_BOTTLE]  = {{3}};       // class 3: water bottle
    task_configs_[DetectionTask::ALL]            = {{1, 2, 3}}; // all objects
}

void ObjectDetector::loadModelForTask(DetectionTask task) {
    if (task == DetectionTask::NONE) {
        return;
    }

    auto it = task_configs_.find(task);
    if (it == task_configs_.end()) {
        std::cerr << "ObjectDetector: Unknown task" << std::endl;
        return;
    }

    // Tokens and class_names_ are global (set once in initializeTaskConfigs)
    // Only the accepted class indices change per task
    std::cout << "ObjectDetector: Switched to task: " << getTaskName(task) << std::endl;
    std::cout << "  Accepting classes:";
    for (int idx : it->second.accepted_class_indices) {
        if (idx >= 0 && static_cast<size_t>(idx) < class_names_.size()) {
            std::cout << " \"" << class_names_[idx] << "\"";
        }
    }
    std::cout << std::endl;
}

void ObjectDetector::setActiveTask(DetectionTask task) {
    if (task == active_task_) {
        return;
    }

    active_task_ = task;

    if (task != DetectionTask::NONE) {
        loadModelForTask(task);
    } else {
        std::cout << "ObjectDetector: All detection tasks disabled" << std::endl;
    }
}

DetectionTask ObjectDetector::getActiveTask() const {
    return active_task_;
}

void ObjectDetector::toggleTask(DetectionTask task) {
    if (active_task_ == task) {
        // Currently active, disable it
        setActiveTask(DetectionTask::NONE);
    } else {
        // Activate this task (automatically disables previous)
        setActiveTask(task);
    }
}

bool ObjectDetector::isEnabled() const {
    return active_task_ != DetectionTask::NONE;
}

std::string ObjectDetector::getTaskName(DetectionTask task) {
    switch (task) {
        case DetectionTask::NONE:          return "NONE";
        case DetectionTask::ORANGE_HAMMER: return "Orange Hammer";
        case DetectionTask::ROCK_PICK:     return "Rock Pick";
        case DetectionTask::WATER_BOTTLE:  return "Water Bottle";
        case DetectionTask::ALL:           return "All Objects";
        default:                           return "Unknown";
    }
}

void ObjectDetector::initUndistortMaps() {
    cv::initUndistortRectifyMap(
        camera_params_.getCameraMatrix(),
        camera_params_.getDistCoeff(),
        cv::Mat_<double>::eye(3, 3),
        camera_params_.getCameraMatrix(),
        camera_params_.getImageSize(),
        CV_16SC2,
        map1_,
        map2_
    );
}

torch::Tensor ObjectDetector::preprocess(const cv::Mat& image) {
    // CLIP normalization (used by OWL-ViT processor during training)
    const std::vector<float> mean = {0.48145466f, 0.4578275f, 0.40821073f};
    const std::vector<float> std = {0.26862954f, 0.26130258f, 0.27577711f};
    
    cv::Mat resized, rgb_image;
    cv::resize(image, resized, cv::Size(768, 768));
    cv::cvtColor(resized, rgb_image, cv::COLOR_BGR2RGB);
    
    // Convert to tensor
    torch::Tensor img_tensor = torch::from_blob(
        rgb_image.data,
        {rgb_image.rows, rgb_image.cols, 3},
        torch::kByte
    );
    
    // Permute to [C, H, W] and convert to float
    img_tensor = img_tensor.permute({2, 0, 1}).to(torch::kFloat32) / 255.0;
    
    // Normalize
    for (int i = 0; i < 3; ++i) {
        img_tensor[i] = (img_tensor[i] - mean[i]) / std[i];
    }
    
    // Add batch dimension
    return img_tensor.unsqueeze(0);
}

std::vector<torch::Tensor> ObjectDetector::runModel(const cv::Mat& image) {
    torch::NoGradGuard no_grad;

    // Preprocess and move to device
    torch::Tensor pixel_values = preprocess(image).to(device_);

    // Run inference (vision-only model: text embeddings are baked in)
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(pixel_values);

    auto outputs = model_.forward(inputs);
    auto output_tuple = outputs.toTuple()->elements();

    // Extract logits and predicted boxes
    auto logits = output_tuple[0].toTensor();
    auto pred_boxes = output_tuple[1].toTensor();

    return {logits, pred_boxes};
}

std::vector<DetectionResult> ObjectDetector::detect(const cv::Mat& image, bool undistort, bool estimate_distance) {
    if (active_task_ == DetectionTask::NONE || empty()) {
        return {};
    }

    // Get accepted class indices for the active task
    auto task_it = task_configs_.find(active_task_);
    if (task_it == task_configs_.end()) {
        return {};
    }
    const auto& accepted = task_it->second.accepted_class_indices;

    cv::Mat processed_image = image;

    // Apply undistortion if requested and camera params available
    if (undistort && !camera_params_.empty()) {
        cv::remap(image, processed_image, map1_, map2_, cv::INTER_LINEAR);
    }

    // Run model (always with all 4 class tokens)
    auto outputs = runModel(processed_image);
    auto logits = outputs[0];
    auto boxes = outputs[1];

    // Convert logits to probabilities (softmax over all 4 classes, matching training)
    torch::Tensor scores = torch::softmax(logits.squeeze(0), 1);

    std::vector<DetectionResult> results;

    int img_width = processed_image.cols;
    int img_height = processed_image.rows;

    // Process each detection
    for (int i = 0; i < scores.size(0); i++) {
        auto max_result = scores[i].max(0);
        float confidence = std::get<0>(max_result).item<float>();
        int class_idx = std::get<1>(max_result).item<int>();

        // Skip background (class 0), low confidence, and classes not in the active task
        if (class_idx == 0 || confidence <= confidence_threshold_) {
            continue;
        }
        if (std::find(accepted.begin(), accepted.end(), class_idx) == accepted.end()) {
            continue;
        }

        auto box = boxes[0][i];

        // Extract normalized coordinates
        float x_center = box[0].item<float>();
        float y_center = box[1].item<float>();
        float width = box[2].item<float>();
        float height = box[3].item<float>();

        // Convert to pixel coordinates
        int x1 = static_cast<int>((x_center - width / 2.0f) * img_width);
        int y1 = static_cast<int>((y_center - height / 2.0f) * img_height);
        int x2 = static_cast<int>((x_center + width / 2.0f) * img_width);
        int y2 = static_cast<int>((y_center + height / 2.0f) * img_height);

        // Clamp to image boundaries
        x1 = std::max(0, std::min(x1, img_width - 1));
        y1 = std::max(0, std::min(y1, img_height - 1));
        x2 = std::max(0, std::min(x2, img_width - 1));
        y2 = std::max(0, std::min(y2, img_height - 1));

        cv::Rect bbox(x1, y1, x2 - x1, y2 - y1);

        // Get class name
        std::string class_name = (static_cast<size_t>(class_idx) < class_names_.size())
                                     ? class_names_[class_idx]
                                     : "unknown";

        // Calculate actual distance if requested
        float distance = -1.0f;
        if (estimate_distance) {
            distance = calculateActualDistance(bbox, class_name);
        }

        results.emplace_back(class_idx, class_name, bbox, confidence, distance);
    }

    // Apply Non-Maximum Suppression to remove overlapping detections
    results = applyNMS(results, 0.5f);

    return results;
}

std::vector<DetectionResult> ObjectDetector::applyNMS(const std::vector<DetectionResult>& detections,
                                                      float nms_threshold) {
    if (detections.empty()) {
        return {};
    }
    
    // Group detections by class
    std::map<int, std::vector<size_t>> class_indices;
    for (size_t i = 0; i < detections.size(); i++) {
        class_indices[detections[i].class_id].push_back(i);
    }
    
    std::vector<DetectionResult> result;
    
    // Apply NMS per class
    for (const auto& pair : class_indices) {
        const std::vector<size_t>& indices = pair.second;
        
        // Sort by confidence (descending)
        std::vector<size_t> sorted_indices = indices;
        std::sort(sorted_indices.begin(), sorted_indices.end(),
                  [&detections](size_t i1, size_t i2) {
                      return detections[i1].confidence > detections[i2].confidence;
                  });
        
        std::vector<bool> suppressed(sorted_indices.size(), false);
        
        for (size_t i = 0; i < sorted_indices.size(); i++) {
            if (suppressed[i]) continue;
            
            const cv::Rect& box1 = detections[sorted_indices[i]].bounding_box;
            result.push_back(detections[sorted_indices[i]]);
            
            // Suppress overlapping boxes
            for (size_t j = i + 1; j < sorted_indices.size(); j++) {
                if (suppressed[j]) continue;
                
                const cv::Rect& box2 = detections[sorted_indices[j]].bounding_box;
                
                // Calculate IoU (Intersection over Union)
                int intersection_x1 = std::max(box1.x, box2.x);
                int intersection_y1 = std::max(box1.y, box2.y);
                int intersection_x2 = std::min(box1.x + box1.width, box2.x + box2.width);
                int intersection_y2 = std::min(box1.y + box1.height, box2.y + box2.height);
                
                int intersection_width = std::max(0, intersection_x2 - intersection_x1);
                int intersection_height = std::max(0, intersection_y2 - intersection_y1);
                int intersection_area = intersection_width * intersection_height;
                
                int box1_area = box1.width * box1.height;
                int box2_area = box2.width * box2.height;
                int union_area = box1_area + box2_area - intersection_area;
                
                float iou = (union_area > 0) ? static_cast<float>(intersection_area) / union_area : 0.0f;
                
                if (iou > nms_threshold) {
                    suppressed[j] = true;
                }
            }
        }
    }
    
    return result;
}

void ObjectDetector::drawDetections(cv::Mat& image, const std::vector<DetectionResult>& results) const {
    for (const auto& result : results) {
        const cv::Rect& bbox = result.bounding_box;
        
        // Draw rectangle
        cv::rectangle(image, bbox, cv::Scalar(0, 255, 0), 2);
        
        // Prepare label text with distance if available
        std::ostringstream label_ss;
        label_ss << result.class_name << " " 
                 << static_cast<int>(result.confidence * 100) << "%";
        
        // Add distance if valid (> 0)
        if (result.actual_distance_meters > 0.0f) {
            label_ss << " " << std::fixed << std::setprecision(2) 
                     << result.actual_distance_meters << "m";
        }
        std::string label = label_ss.str();
        
        // Get text size for background
        int baseline;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // Draw filled rectangle for text background
        cv::rectangle(image, 
                     cv::Point(bbox.x, bbox.y - text_size.height - baseline),
                     cv::Point(bbox.x + text_size.width, bbox.y),
                     cv::Scalar(0, 255, 0),
                     cv::FILLED);
        
        // Draw label text
        cv::putText(image, label,
                   cv::Point(bbox.x, bbox.y - 5),
                   cv::FONT_HERSHEY_SIMPLEX,
                   0.5,
                   cv::Scalar(0, 0, 0),
                   1);
    }
}

void ObjectDetector::setConfidenceThreshold(float threshold) {
    confidence_threshold_ = threshold;
}

float ObjectDetector::getConfidenceThreshold() const {
    return confidence_threshold_;
}

bool ObjectDetector::empty() const {
    return class_names_.empty();
}

void ObjectDetector::initializeObjectDimensions() {
    // Handled globally in initializeTaskConfigs()
}

float ObjectDetector::calculateActualDistance(const cv::Rect& bbox, const std::string& class_name) const {
    // Check if we have camera calibration
    if (camera_params_.empty()) {
        return -1.0f;
    }
    
    // Check if we know this object's dimensions
    auto it_height = object_heights_.find(class_name);
    auto it_width = object_widths_.find(class_name);
    
    if (it_height == object_heights_.end()) {
        return -1.0f;
    }
    
    float real_height_meters = it_height->second;
    
    // Get focal lengths from camera matrix
    // Camera matrix format: [fx, 0, cx]
    //                       [0, fy, cy]
    //                       [0,  0,  1]
    cv::Mat camera_matrix = camera_params_.getCameraMatrix();
    float focal_length_x = camera_matrix.at<double>(0, 0);  // fx
    float focal_length_y = camera_matrix.at<double>(1, 1);  // fy
    
    // Pixel dimensions of the object
    float pixel_height = static_cast<float>(bbox.height);
    float pixel_width = static_cast<float>(bbox.width);
    
    if (pixel_height < 1.0f) {
        return -1.0f;  // Too small to measure accurately
    }
    
    // Calculate distance from height
    float distance_from_height = (real_height_meters * focal_length_y) / pixel_height;
    
    // Calculate distance from width if available
    if (it_width != object_widths_.end() && pixel_width >= 1.0f) {
        float real_width_meters = it_width->second;
        float distance_from_width = (real_width_meters * focal_length_x) / pixel_width;
        
        // Average of both measurements for better accuracy
        return (distance_from_height + distance_from_width) / 2.0f;
    }
    
    // Fall back to height-only calculation
    return distance_from_height;
}

float ObjectDetector::getDistanceFromDepth(const cv::Mat& depth_frame, float depth_scale,
                                            const cv::Rect& bbox) {
    if (depth_frame.empty() || depth_frame.type() != CV_16UC1) {
        return -1.0f;
    }
    
    // Sample from center region (inner 50% of bbox) for robustness
    int cx = bbox.x + bbox.width / 2;
    int cy = bbox.y + bbox.height / 2;
    int sample_w = bbox.width / 4;
    int sample_h = bbox.height / 4;
    
    int x1 = std::max(0, cx - sample_w);
    int y1 = std::max(0, cy - sample_h);
    int x2 = std::min(depth_frame.cols - 1, cx + sample_w);
    int y2 = std::min(depth_frame.rows - 1, cy + sample_h);
    
    // Collect valid depth values
    std::vector<float> depths;
    depths.reserve((x2 - x1 + 1) * (y2 - y1 + 1));
    
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            uint16_t raw = depth_frame.at<uint16_t>(y, x);
            if (raw > 0) {
                depths.push_back(static_cast<float>(raw) * depth_scale);
            }
        }
    }
    
    if (depths.empty()) {
        return -1.0f;  // No valid depth data in region
    }
    
    // Return median for robustness against outliers
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    return depths[depths.size() / 2];
}

std::vector<DetectionResult> ObjectDetector::detectWithDepth(const cv::Mat& color_image,
                                                              const cv::Mat& depth_image,
                                                              float depth_scale,
                                                              bool undistort) {
    // Run detection on color image (without distance estimation from pinhole model)
    std::vector<DetectionResult> results = detect(color_image, undistort, false);
    
    // Limit to max 3 results (keep highest confidence)
    const size_t MAX_DETECTIONS = 3;
    if (results.size() > MAX_DETECTIONS) {
        // Find indices of top 3 by confidence
        std::vector<size_t> indices(results.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::partial_sort(indices.begin(), indices.begin() + MAX_DETECTIONS, indices.end(),
                         [&results](size_t a, size_t b) {
                             return results[a].confidence > results[b].confidence;
                         });
        
        std::vector<DetectionResult> top_results;
        top_results.reserve(MAX_DETECTIONS);
        for (size_t i = 0; i < MAX_DETECTIONS; ++i) {
            top_results.push_back(std::move(results[indices[i]]));
        }
        results = std::move(top_results);
    }
    
    // Update distances using depth image
    for (auto& result : results) {
        result.actual_distance_meters = getDistanceFromDepth(depth_image, depth_scale, 
                                                              result.bounding_box);
    }
    
    return results;
}

} // namespace ObjDet
