#include "ObjectDetector.h"
#include "ModelDownloader.h"
#include "../camera/CameraParams.h"

#ifdef WITH_REALSENSE
#include "../camera/RealSenseCamera.h"
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace ObjDet;

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
    std::cout << "=== RealSense D455 Object Detection Test ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  '1' - Toggle Orange Hammer detection" << std::endl;
    std::cout << "  '2' - Toggle Rock Pick detection" << std::endl;
    std::cout << "  '3' - Toggle Water Bottle detection" << std::endl;
    std::cout << "  '4' - Toggle All Objects detection" << std::endl;
    std::cout << "  '0' - Disable all detection" << std::endl;
    std::cout << "  'd' - Toggle depth overlay" << std::endl;
    std::cout << "  '+' - Increase confidence threshold" << std::endl;
    std::cout << "  '-' - Decrease confidence threshold" << std::endl;
    std::cout << "  'q' - Quit" << std::endl;
    std::cout << std::endl;

#ifndef WITH_REALSENSE
    std::cerr << "Error: RealSense support not compiled in!" << std::endl;
    std::cerr << "Rebuild with: cmake -DWITH_REALSENSE=ON -DWORLD_INTERFACE=REAL .." << std::endl;
    return 1;
#else
    try {
        // Initialize RealSense camera
        std::cout << "Initializing RealSense D455..." << std::endl;
        cam::RealSenseCamera camera;
        
        if (!camera.start(1280, 720, 30)) {
            std::cerr << "Error: Failed to start RealSense camera" << std::endl;
            return 1;
        }
        
        std::cout << "RealSense camera started successfully" << std::endl;
        std::cout << "Depth scale: " << camera.getDepthScale() << " m/unit" << std::endl;
        
        // Get camera intrinsic parameters from RealSense
        cam::CameraParams camera_params;
        if (camera.hasIntrinsicParams()) {
            camera_params = camera.getIntrinsicParams();
            std::cout << "Intrinsic parameters loaded from camera" << std::endl;
        }

        // Find model (auto-downloads from HuggingFace if not found)
        std::string model_path = findOrDownloadModel();
        std::cout << "Loading model from: " << model_path << std::endl;
        ObjectDetector detector(model_path, 0.75f, camera_params);  // 75% confidence threshold
        std::cout << "Model loaded successfully" << std::endl;

        // Create window (1280x720 to match camera resolution)
        cv::namedWindow("RealSense Detection", cv::WINDOW_NORMAL);
        cv::resizeWindow("RealSense Detection", 1280, 720);
        
        bool show_depth_overlay = false;
        uint32_t last_frame_num = 0;
        
        // FPS tracking
        auto fps_start = std::chrono::steady_clock::now();
        int frame_count = 0;
        float fps = 0.0f;
        
        std::cout << "\nStarting detection loop. Press 'q' to quit.\n" << std::endl;
        std::cout << "Press 'd' to toggle depth overlay.\n" << std::endl;

        while (true) {
            // Wait for new frame
            if (!camera.hasNext(last_frame_num)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            
            // Get frames
            cv::Mat color_frame, depth_frame;
            robot::types::datatime_t timestamp;
            
            if (!camera.getFrames(color_frame, depth_frame, last_frame_num, timestamp)) {
                continue;
            }
            
            // Calculate FPS
            frame_count++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_start).count();
            if (elapsed >= 1000) {
                fps = frame_count * 1000.0f / elapsed;
                frame_count = 0;
                fps_start = now;
            }
            
            // Create display copy
            cv::Mat display = color_frame.clone();
            
            // Run detection if enabled
            if (detector.isEnabled()) {
                // Use depth-based distance measurement
                auto results = detector.detectWithDepth(color_frame, depth_frame, 
                                                         camera.getDepthScale());
                
                // Draw detections with depth sampling region visualization
                for (const auto& result : results) {
                    const cv::Rect& bbox = result.bounding_box;
                    
                    // Draw main bounding box
                    cv::rectangle(display, bbox, cv::Scalar(0, 255, 0), 2);
                    
                    // Draw depth sampling region (inner 50% of bbox)
                    int cx = bbox.x + bbox.width / 2;
                    int cy = bbox.y + bbox.height / 2;
                    int sample_w = bbox.width / 4;
                    int sample_h = bbox.height / 4;
                    cv::Rect sample_region(cx - sample_w, cy - sample_h, sample_w * 2, sample_h * 2);
                    cv::rectangle(display, sample_region, cv::Scalar(255, 0, 255), 1);  // Magenta sampling region
                    
                    // Prepare label with distance
                    std::ostringstream label_ss;
                    label_ss << result.class_name << " " 
                             << static_cast<int>(result.confidence * 100) << "%";
                    if (result.actual_distance_meters > 0.0f) {
                        label_ss << " " << std::fixed << std::setprecision(2) 
                                 << result.actual_distance_meters << "m";
                    } else {
                        label_ss << " (no depth)";
                    }
                    std::string label = label_ss.str();
                    
                    // Draw label background
                    int baseline;
                    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
                    cv::rectangle(display, 
                                 cv::Point(bbox.x, bbox.y - text_size.height - baseline - 5),
                                 cv::Point(bbox.x + text_size.width, bbox.y),
                                 cv::Scalar(0, 255, 0), cv::FILLED);
                    
                    // Draw label text
                    cv::putText(display, label, cv::Point(bbox.x, bbox.y - 5),
                               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
                }
                
                // Print detections to console
                for (const auto& result : results) {
                    std::cout << "Detected: " << result.class_name 
                              << " (conf=" << std::fixed << std::setprecision(2) << result.confidence
                              << ", dist=" << std::setprecision(2) << result.actual_distance_meters << "m)"
                              << std::endl;
                }
            }
            
            // Draw task info
            std::string task_str = "Task: " + ObjectDetector::getTaskName(detector.getActiveTask());
            cv::putText(display, task_str, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // Draw FPS
            std::stringstream fps_ss;
            fps_ss << "FPS: " << std::fixed << std::setprecision(1) << fps;
            cv::putText(display, fps_ss.str(), cv::Point(10, 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // Draw confidence threshold
            std::stringstream conf_ss;
            conf_ss << "Threshold: " << std::fixed << std::setprecision(2) 
                    << detector.getConfidenceThreshold();
            cv::putText(display, conf_ss.str(), cv::Point(10, 90),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // Overlay depth on color if enabled
            if (show_depth_overlay) {
                cv::Mat depth_colorized;
                if (camera.getColorizedDepth(depth_colorized)) {
                    cv::cvtColor(depth_colorized, depth_colorized, cv::COLOR_RGB2BGR);
                    cv::addWeighted(display, 0.6, depth_colorized, 0.4, 0, display);
                }
            }
            
            // Show frame
            cv::imshow("RealSense Detection", display);
            
            // Handle keyboard input
            int key = cv::waitKey(1) & 0xFF;
            
            if (key == 'q' || key == 27) {
                break;
            } else if (key == '1') {
                detector.toggleTask(DetectionTask::ORANGE_HAMMER);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '2') {
                detector.toggleTask(DetectionTask::ROCK_PICK);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '3') {
                detector.toggleTask(DetectionTask::WATER_BOTTLE);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '4') {
                detector.toggleTask(DetectionTask::ALL);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '0') {
                detector.setActiveTask(DetectionTask::NONE);
                std::cout << "Detection disabled" << std::endl;
            } else if (key == 'd') {
                show_depth_overlay = !show_depth_overlay;
                std::cout << "Depth overlay: " << (show_depth_overlay ? "ON" : "OFF") << std::endl;
            } else if (key == '+' || key == '=') {
                float thresh = detector.getConfidenceThreshold();
                detector.setConfidenceThreshold(std::min(0.99f, thresh + 0.05f));
                std::cout << "Threshold: " << detector.getConfidenceThreshold() << std::endl;
            } else if (key == '-') {
                float thresh = detector.getConfidenceThreshold();
                detector.setConfidenceThreshold(std::max(0.01f, thresh - 0.05f));
                std::cout << "Threshold: " << detector.getConfidenceThreshold() << std::endl;
            }
        }
        
        camera.stop();
        cv::destroyAllWindows();
        std::cout << "Done." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
#endif
}
