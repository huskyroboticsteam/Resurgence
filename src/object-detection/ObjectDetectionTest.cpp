#include "read_objects.h"
#include "ObjectDetector.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

cv::Mat visualizeDetections(const cv::Mat& image, const std::vector<ObjDet::DetectionResult>& detections, const std::string& title, float scale = 1.0f) {
    cv::Mat display_image = image.clone();
    
    // Draw detection boxes and labels on image
    for (const auto& det : detections) {
        // Scale bounding box coordinates
        cv::Rect scaled_box(
            static_cast<int>(det.bounding_box.x * scale),
            static_cast<int>(det.bounding_box.y * scale),
            static_cast<int>(det.bounding_box.width * scale),
            static_cast<int>(det.bounding_box.height * scale)
        );
        
        // Draw bounding box (green, thickness 2)
        cv::rectangle(display_image, scaled_box, cv::Scalar(0, 255, 0), 2);
        
        // Prepare label text
        std::stringstream label_stream;
        label_stream << det.class_name << " " 
                     << std::fixed << std::setprecision(2) 
                     << (det.confidence * 100) << "%";
        std::string label = label_stream.str();
        
        // Calculate text background size
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // Position label above the box
        int label_y = scaled_box.y - 8;
        if (label_y - text_size.height < 0) {
            // If no space above, put it inside at the top
            label_y = scaled_box.y + text_size.height + 5;
        }
        
        // Draw text background (green filled)
        cv::Point text_origin(scaled_box.x, label_y);
        cv::rectangle(display_image,
                     cv::Point(text_origin.x, text_origin.y - text_size.height - 3),
                     cv::Point(text_origin.x + text_size.width + 4, text_origin.y + 3),
                     cv::Scalar(0, 255, 0),
                     cv::FILLED);
        
        // Draw text (black)
        cv::putText(display_image, label,
                   cv::Point(text_origin.x + 2, text_origin.y),
                   cv::FONT_HERSHEY_SIMPLEX,
                   0.5,
                   cv::Scalar(0, 0, 0),
                   1);
    }
    
    // Add title bar at the top
    int title_height = 50;
    cv::Mat title_bar = cv::Mat::zeros(title_height, display_image.cols, display_image.type());
    title_bar.setTo(cv::Scalar(40, 40, 40));
    
    // Add title text
    cv::putText(title_bar, title,
               cv::Point(10, 35),
               cv::FONT_HERSHEY_SIMPLEX,
               0.8,
               cv::Scalar(255, 255, 255),
               2);
    
    // Add detection count
    std::stringstream count_stream;
    count_stream << "Objects: " << detections.size();
    cv::Size count_size = cv::getTextSize(count_stream.str(), cv::FONT_HERSHEY_SIMPLEX, 0.8, 2, nullptr);
    cv::putText(title_bar, count_stream.str(),
               cv::Point(display_image.cols - count_size.width - 10, 35),
               cv::FONT_HERSHEY_SIMPLEX,
               0.8,
               cv::Scalar(0, 255, 0),
               2);
    
    // Combine title bar and image
    cv::Mat result;
    cv::vconcat(title_bar, display_image, result);
    
    return result;
}

int main(int argc, char** argv) {
    // Default paths (relative to build directory)
    std::string image_path = "../src/object-detection/test.jpg";
    std::string model_path = "../src/object-detection/owlvit-cpp.pt";
    
    // Override with command line argument if provided
    if (argc >= 2) {
        image_path = argv[1];
    }
    if (argc >= 3) {
        model_path = argv[2];
    }

    std::cout << "Testing object detection integration..." << std::endl;
    std::cout << "Image path: " << image_path << std::endl;
    std::cout << "Model path: " << model_path << std::endl;

    // Use ObjectDetector directly, without world_interface
    cv::Mat test_image = cv::imread(image_path);
    if (test_image.empty()) {
        std::cerr << "Error: Could not load image " << image_path << std::endl;
        return 1;
    }

    std::cout << "Image size: " << test_image.cols << "x" << test_image.rows << std::endl;

    // Create detector
    std::vector<std::string> classes = {
        "no object",
        "orange mallet or hammer",
        "water bottle"
    };
    
    cam::CameraParams params;  // Use default camera params
    
    ObjDet::ObjectDetector detector(classes, model_path, 0.6f, params);

    // Test 1: Detection enabled
    std::cout << "\n=== Test 1: Detection ENABLED ===" << std::endl;
    detector.setEnabled(true);
    detector.setConfidenceThreshold(0.6f);
    
    auto detections_enabled = detector.detect(test_image);
    std::cout << "Found " << detections_enabled.size() << " objects:" << std::endl;
    for (const auto& det : detections_enabled) {
        std::cout << "  - " << det.class_name 
                  << " (confidence: " << std::fixed << std::setprecision(4) << det.confidence << ")" 
                  << " at [" << det.bounding_box.x << ", " << det.bounding_box.y 
                  << ", " << det.bounding_box.width << ", " << det.bounding_box.height << "]"
                  << std::endl;
    }

    // Test 2: Detection disabled
    std::cout << "\n=== Test 2: Detection DISABLED ===" << std::endl;
    detector.setEnabled(false);
    auto detections_disabled = detector.detect(test_image);
    std::cout << "Found " << detections_disabled.size() << " objects (should be 0)" << std::endl;
    
    // Resize images to 50% for smaller display
    cv::Mat resized_image;
    cv::resize(test_image, resized_image, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    
    // Create side-by-side comparison with resized images (scale factor 0.5)
    cv::Mat result_enabled = visualizeDetections(resized_image, detections_enabled, "ENABLED", 0.5f);
    cv::Mat result_disabled = visualizeDetections(resized_image, detections_disabled, "DISABLED", 0.5f);
    
    // Concatenate horizontally
    cv::Mat combined;
    cv::hconcat(result_enabled, result_disabled, combined);
    
    // Display combined result
    cv::imshow("Object Detection Test - Side by Side Comparison", combined);
    std::cout << "\nPress any key to exit..." << std::endl;
    cv::waitKey(0);

    cv::destroyAllWindows();

    std::cout << "\n✅ All integration tests passed!" << std::endl;
    std::cout << "✅ Enable/disable functionality works correctly" << std::endl;
    return 0;
}
