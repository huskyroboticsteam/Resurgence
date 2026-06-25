#include "ObjectDetector.h"
#include "ModelDownloader.h"
#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
#include "../Constants.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <cstdlib>

using namespace ObjDet;

// Prompt user to download model
bool promptDownload(const std::string& destination) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "OWL-ViT Model Not Found" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "The object detection model is required but not found." << std::endl;
    std::cout << "Would you like to download it now? (y/n): ";
    
    std::string response;
    std::getline(std::cin, response);
    
    if (response == "y" || response == "Y" || response == "yes" || response == "Yes") {
        return downloadModel(destination);
    } else {
        std::cout << "Download cancelled." << std::endl;
        return false;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== Object Detection Camera Test ===" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  '1' - Toggle Orange Hammer detection" << std::endl;
    std::cout << "  '2' - Toggle Rock Pick detection" << std::endl;
    std::cout << "  '3' - Toggle Water Bottle detection" << std::endl;
    std::cout << "  '4' - Toggle All Objects detection" << std::endl;
    std::cout << "  '0' - Disable all detection" << std::endl;
    std::cout << "  '+' - Increase confidence threshold" << std::endl;
    std::cout << "  '-' - Decrease confidence threshold" << std::endl;
    std::cout << "  'q' - Quit" << std::endl;
    std::cout << std::endl;

    // Parse command line arguments
    int camera_id = 0;  // Default to first camera
    if (argc > 1) {
        camera_id = std::atoi(argv[1]);
    }

    try {
        // Initialize camera
        std::cout << "Opening camera " << camera_id << "..." << std::endl;
        cv::VideoCapture cap(camera_id);
        
        if (!cap.isOpened()) {
            std::cerr << "Error: Cannot open camera " << camera_id << std::endl;
            std::cerr << "Usage: " << argv[0] << " [camera_id]" << std::endl;
            return 1;
        }

        // Set camera to use MJPEG format (better color support)
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        
        // Set camera resolution (optional, adjust as needed)
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        std::cout << "Camera opened successfully" << std::endl;
        std::cout << "Resolution: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) 
                  << "x" << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

        // Capture a test frame to check format
        cv::Mat test_frame;
        cap >> test_frame;
        if (!test_frame.empty()) {
            std::cout << "Image format: " << test_frame.channels() << " channels (";
            if (test_frame.channels() == 1) {
                std::cout << "Grayscale";
            } else if (test_frame.channels() == 3) {
                std::cout << "Color BGR";
            } else if (test_frame.channels() == 4) {
                std::cout << "Color BGRA";
            }
            std::cout << ")" << std::endl;
        }

        // Load camera configuration for intrinsic parameters
        std::cout << "Loading camera configuration..." << std::endl;
        cam::CameraParams camera_params;
        
        bool params_loaded = false;
        try {
            auto config = cam::readConfigFromFile(
                Constants::CAMERA_CONFIG_PATHS.at(Constants::MAST_CAMERA_ID)
            );
            
            if (config.intrinsicParams && !config.intrinsicParams->empty()) {
                camera_params = config.intrinsicParams.value();
                std::cout << "Camera intrinsic parameters loaded" << std::endl;
                params_loaded = true;
            }
        } catch (const std::exception& e) {
            std::cout << "Warning: Could not load camera config: " << e.what() << std::endl;
        }
        
        // Create default camera parameters if not loaded
        if (!params_loaded) {
            std::cout << "Creating default camera parameters..." << std::endl;
            // Camera 4 calibrated parameters (from calibrate_camera tool)
            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 
                202.5, 0, 640,    // fx, 0, cx
                0, 202.5, 400,    // 0, fy, cy
                0, 0, 1);         // 0, 0, 1
            cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
            camera_params = cam::CameraParams(camera_matrix, dist_coeffs, cv::Size(1280, 800));
            std::cout << "Using calibrated focal length: 202.5px" << std::endl;
        }

        // Initialize object detector
        std::cout << "Initializing object detector..." << std::endl;

        // Find or download model file
        std::string model_path;
        try {
            model_path = findOrDownloadModel();
        } catch (const std::exception& e) {
            // Model not found - prompt user to download
            std::cerr << "Warning: " << e.what() << std::endl;
            
            // Determine best download location
            std::string download_path;
            if (std::filesystem::exists("../src/object-detection")) {
                download_path = "../src/object-detection/owlvit_finetune.pt";
            } else if (std::filesystem::exists("src/object-detection")) {
                download_path = "src/object-detection/owlvit_finetune.pt";
            } else {
                download_path = "owlvit_finetune.pt";
            }
            
            // Prompt user to download
            if (promptDownload(download_path)) {
                model_path = download_path;
            } else {
                std::cerr << "\nYou can also set OWLVIT_MODEL_PATH environment variable" << std::endl;
                return 1;
            }
        }

        std::cout << "Using model: " << model_path << std::endl;

        ObjectDetector detector(model_path, 0.6f, camera_params);
        std::cout << "Object detector initialized" << std::endl;
        std::cout << std::endl;

        // Create window with larger size
        const std::string window_name = "Object Detection Test";
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name, 1280, 720);  // Set to HD size

        float confidence_threshold = 0.9f;
        cv::Mat frame;

        // FPS calculation
        auto last_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        double fps = 0.0;

        while (true) {
            // Capture frame
            cap >> frame;
            if (frame.empty()) {
                std::cerr << "Error: Empty frame" << std::endl;
                break;
            }

            // Convert to BGR if grayscale (for consistent display)
            if (frame.channels() == 1) {
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
            }

            cv::Mat display_frame = frame.clone();

            // Run detection if a task is active
            std::vector<DetectionResult> detections;
            auto detect_start = std::chrono::steady_clock::now();
            
            DetectionTask current_task = detector.getActiveTask();
            if (current_task != DetectionTask::NONE) {
                detector.setConfidenceThreshold(confidence_threshold);
                detections = detector.detect(frame, false, true);  // undistort=false, estimate_distance=true
            }
            
            auto detect_end = std::chrono::steady_clock::now();
            double detect_time = std::chrono::duration<double, std::milli>(detect_end - detect_start).count();

            // Draw detections
            if (current_task != DetectionTask::NONE && !detections.empty()) {
                for (const auto& det : detections) {
                    // Draw bounding box
                    cv::rectangle(display_frame, det.bounding_box, cv::Scalar(0, 255, 0), 2);
                    
                    // Prepare label with distance
                    std::stringstream label_stream;
                    label_stream << det.class_name << " " 
                               << std::fixed << std::setprecision(1) 
                               << (det.confidence * 100) << "%";
                    
                    // Add distance if available
                    if (det.actual_distance_meters >= 0.0f) {
                        label_stream << " [" << std::setprecision(2) 
                                    << det.actual_distance_meters << "m]";
                    }
                    std::string label = label_stream.str();
                    
                    // Draw label background
                    int baseline = 0;
                    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
                    
                    cv::Point text_origin(det.bounding_box.x, det.bounding_box.y - 8);
                    if (text_origin.y < text_size.height) {
                        text_origin.y = det.bounding_box.y + text_size.height + 8;
                    }
                    
                    cv::rectangle(display_frame,
                                cv::Point(text_origin.x - 2, text_origin.y - text_size.height - 4),
                                cv::Point(text_origin.x + text_size.width + 2, text_origin.y + 4),
                                cv::Scalar(0, 255, 0),
                                cv::FILLED);
                    
                    // Draw label text
                    cv::putText(display_frame, label, text_origin,
                              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
                }
            }

            // Calculate FPS
            frame_count++;
            auto current_time = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(current_time - last_time).count();
            if (elapsed >= 1.0) {
                fps = frame_count / elapsed;
                frame_count = 0;
                last_time = current_time;
            }

            // Draw info overlay
            int y_offset = 30;
            cv::putText(display_frame, 
                       "FPS: " + std::to_string(static_cast<int>(fps)),
                       cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(0, 255, 0), 2);
            y_offset += 30;

            // Show current task
            std::string task_name = ObjectDetector::getTaskName(current_task);
            if (current_task != DetectionTask::NONE) {
                cv::putText(display_frame, 
                           "Task: " + task_name,
                           cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(0, 255, 0), 2);
                y_offset += 30;
                
                cv::putText(display_frame, 
                           "Detection time: " + std::to_string(static_cast<int>(detect_time)) + "ms",
                           cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(0, 255, 0), 2);
                y_offset += 30;
                
                cv::putText(display_frame, 
                           "Objects: " + std::to_string(detections.size()),
                           cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(0, 255, 0), 2);
                y_offset += 30;
            } else {
                cv::putText(display_frame, 
                           "Detection: OFF (Press 1/2/3 to enable)",
                           cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                           cv::Scalar(0, 0, 255), 2);
                y_offset += 30;
            }

            cv::putText(display_frame, 
                       "Confidence: " + std::to_string(static_cast<int>(confidence_threshold * 100)) + "%",
                       cv::Point(10, y_offset), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                       cv::Scalar(255, 255, 0), 2);
            
            // Draw task legend at bottom
            int legend_y = display_frame.rows - 20;
            cv::putText(display_frame, 
                       "[1]Orange Hammer  [2]Rock Pick  [3]Water Bottle  [0]Off",
                       cv::Point(10, legend_y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                       cv::Scalar(200, 200, 200), 1);

            // Show frame
            cv::imshow(window_name, display_frame);

            // Handle keyboard input
            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q' || key == 27) {  // 'q' or ESC
                break;
            } else if (key == '1') {  // Toggle Orange Hammer
                detector.toggleTask(DetectionTask::ORANGE_HAMMER);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '2') {  // Toggle Rock Pick
                detector.toggleTask(DetectionTask::ROCK_PICK);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '3') {  // Toggle Water Bottle
                detector.toggleTask(DetectionTask::WATER_BOTTLE);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '4') {  // Toggle All Objects
                detector.toggleTask(DetectionTask::ALL);
                std::cout << "Task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
            } else if (key == '0') {  // Disable all
                detector.setActiveTask(DetectionTask::NONE);
                std::cout << "Detection disabled" << std::endl;
            } else if (key == '+' || key == '=') {  // Increase threshold
                confidence_threshold = std::min(0.95f, confidence_threshold + 0.05f);
                std::cout << "Confidence threshold: " << (confidence_threshold * 100) << "%" << std::endl;
            } else if (key == '-' || key == '_') {  // Decrease threshold
                confidence_threshold = std::max(0.1f, confidence_threshold - 0.05f);
                std::cout << "Confidence threshold: " << (confidence_threshold * 100) << "%" << std::endl;
            }
        }

        cap.release();
        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Test completed" << std::endl;
    return 0;
}