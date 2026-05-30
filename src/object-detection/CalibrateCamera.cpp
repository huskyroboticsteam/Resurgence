#include "ObjectDetector.h"
#include "../camera/CameraParams.h"
#include "../world_interface/world_interface.h"
#include "../Constants.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

using namespace ObjDet;

int main(int argc, char** argv) {
    std::cout << "=== Quick Camera Calibration Tool ===" << std::endl;
    std::cout << std::endl;
    std::cout << "Instructions:" << std::endl;
    std::cout << "1. Place object at EXACTLY 0.3 meter (30 cm) from camera" << std::endl;
    std::cout << "2. Make sure the object is clearly visible and upright" << std::endl;
    std::cout << "3. Press 'c' to capture and calculate focal length" << std::endl;
    std::cout << "4. Press 'q' to quit" << std::endl;
    std::cout << std::endl;
    std::cout << "Known object heights:" << std::endl;
    std::cout << "  - Water bottle: 0.20 m (20 cm)" << std::endl;
    std::cout << "  - Red/Orange mallet: 0.30 m (30 cm)" << std::endl;
    std::cout << std::endl;
    std::cout << "Usage: calibrate_camera <camera_id_or_name> [distance] [object_type] [--sim]" << std::endl;
    std::cout << "  --sim: Use simulator camera instead of local camera" << std::endl;
    std::cout << std::endl;

    // Parse command line arguments
    bool use_simulator = false;
    std::string camera_arg;
    int camera_id = 0;
    robot::types::CameraID camera_name;
    
    // Check for --sim flag
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--sim") {
            use_simulator = true;
        }
    }
    
    if (argc > 1 && std::string(argv[1]) != "--sim") {
        camera_arg = argv[1];
        // Try to parse as integer first
        try {
            camera_id = std::stoi(camera_arg);
        } catch (...) {
            // If not an integer, treat as camera name (e.g., "mast")
            camera_name = camera_arg;
        }
    }

    float known_distance = 0.3f;  // meters (30 cm)
    if (argc > 2 && std::string(argv[2]) != "--sim") {
        known_distance = std::atof(argv[2]);
    }

    std::string object_type = "water bottle";
    if (argc > 3 && std::string(argv[3]) != "--sim") {
        object_type = argv[3];
    }

    std::cout << "Mode: " << (use_simulator ? "SIMULATOR" : "LOCAL CAMERA") << std::endl;
    if (use_simulator) {
        std::cout << "Camera: " << (!camera_arg.empty() ? camera_arg : "default") << std::endl;
    } else {
        std::cout << "Camera ID: " << camera_id << std::endl;
    }
    std::cout << "Calibration distance: " << known_distance << " m" << std::endl;
    std::cout << "Object type: " << object_type << std::endl;
    std::cout << std::endl;

    // Object heights in meters
    float object_height;
    if (object_type == "water bottle" || object_type == "bottle") {
        object_height = 0.20f;  // 20 cm
    } else if (object_type.find("mallet") != std::string::npos || 
               object_type.find("hammer") != std::string::npos) {
        object_height = 0.30f;  // 30 cm
    } else {
        std::cerr << "Unknown object type. Using default 0.20m" << std::endl;
        object_height = 0.20f;
    }

    std::cout << "Object height: " << object_height << " m" << std::endl;
    std::cout << std::endl;

    try {
        // Open camera (local or simulator)
        cv::VideoCapture cap;
        std::shared_ptr<robot::types::CameraHandle> sim_camera;
        uint32_t last_frame_no = 0;
        int img_width = 1280;
        int img_height = 720;
        
        if (use_simulator) {
            std::cout << "Connecting to simulator camera..." << std::endl;
            if (!camera_arg.empty()) {
                sim_camera = robot::openCamera(camera_name);
            } else {
                // Default to mast camera
                sim_camera = robot::openCamera(Constants::MAST_CAMERA_ID);
                camera_name = Constants::MAST_CAMERA_ID;
            }
            
            if (!sim_camera) {
                std::cerr << "Error: Cannot open simulator camera" << std::endl;
                return 1;
            }
            
            std::cout << "Waiting for first frame from simulator..." << std::endl;
            // Wait for first frame
            while (!robot::hasNewCameraFrame(camera_name, last_frame_no)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            
            auto first_frame = robot::readCamera(camera_name);
            if (first_frame.isValid()) {
                cv::Mat frame = first_frame.getData().first;
                img_width = frame.cols;
                img_height = frame.rows;
                last_frame_no = first_frame.getData().second;
            }
            std::cout << "Simulator camera connected!" << std::endl;
        } else {
            std::cout << "Opening local camera..." << std::endl;
            cap.open(camera_id);
            if (!cap.isOpened()) {
                std::cerr << "Error: Cannot open camera " << camera_id << std::endl;
                return 1;
            }

            cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
            cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

            img_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            img_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        }
        
        std::cout << "Camera resolution: " << img_width << "x" << img_height << std::endl;

        // Load detector
        std::string model_path;
        std::vector<std::string> possible_paths = {
            "src/object-detection/owlvit-cpp.pt",
            "../src/object-detection/owlvit-cpp.pt",
            "owlvit-cpp.pt"
        };

        for (const auto& path : possible_paths) {
            if (std::ifstream(path).good()) {
                model_path = path;
                break;
            }
        }

        if (model_path.empty()) {
            const char* env_path = std::getenv("OWLVIT_MODEL_PATH");
            if (env_path) {
                model_path = env_path;
            } else {
                std::cerr << "Error: Cannot find model file" << std::endl;
                return 1;
            }
        }

        // Create temporary camera params (we're just detecting, not calculating distance yet)
        cv::Mat temp_camera_matrix = (cv::Mat_<double>(3, 3) << 
            800, 0, img_width/2.0,
            0, 800, img_height/2.0,
            0, 0, 1);
        cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
        cam::CameraParams temp_params(temp_camera_matrix, dist_coeffs, cv::Size(img_width, img_height));

        ObjectDetector detector(model_path, 0.9f, temp_params);
        
        // Set the appropriate task based on object type
        if (object_type == "water bottle" || object_type == "bottle") {
            detector.setActiveTask(DetectionTask::WATER_BOTTLE);
        } else if (object_type.find("orange") != std::string::npos) {
            detector.setActiveTask(DetectionTask::ORANGE_HAMMER);
        } else {
            detector.setActiveTask(DetectionTask::ROCK_PICK);
        }
        
        std::cout << "Detector initialized with task: " << ObjectDetector::getTaskName(detector.getActiveTask()) << std::endl;
        std::cout << std::endl;

        cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
        cv::resizeWindow("Calibration", 1280, 720);

        cv::Mat frame;
        bool calibrated = false;
        float calculated_focal_length = 0.0f;

        while (true) {
            // Read frame based on mode
            if (use_simulator) {
                if (robot::hasNewCameraFrame(camera_name, last_frame_no)) {
                    auto cam_data = robot::readCamera(camera_name);
                    if (cam_data.isValid()) {
                        frame = cam_data.getData().first.clone();
                        last_frame_no = cam_data.getData().second;
                    } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        continue;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                }
            } else {
                cap >> frame;
            }
            
            if (frame.empty()) break;

            cv::Mat display = frame.clone();

            // Detect objects
            auto detections = detector.detect(frame, false, false);  // No distance calc yet

            // Draw detections
            for (const auto& det : detections) {
                cv::rectangle(display, det.bounding_box, cv::Scalar(0, 255, 0), 2);
                
                std::string label = det.class_name + " " + 
                                   std::to_string(static_cast<int>(det.confidence * 100)) + "%";
                
                cv::putText(display, label,
                           cv::Point(det.bounding_box.x, det.bounding_box.y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            }

            // Draw instructions
            cv::putText(display, "Press 'c' to calibrate with visible object",
                       cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                       cv::Scalar(0, 255, 255), 2);
            
            cv::putText(display, "Object must be at " + std::to_string(known_distance) + "m distance",
                       cv::Point(20, 65), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                       cv::Scalar(0, 255, 255), 2);

            if (calibrated) {
                cv::putText(display, "Calibrated! Focal length: " + 
                           std::to_string(static_cast<int>(calculated_focal_length)) + "px",
                           cv::Point(20, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, 
                           cv::Scalar(0, 255, 0), 2);
            }

            cv::imshow("Calibration", display);

            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') {
                break;
            } else if (key == 'c' || key == 'C') {
                // Calibrate
                if (detections.empty()) {
                    std::cout << "No objects detected! Please ensure object is visible." << std::endl;
                    continue;
                }

                // Find the largest detection (closest to camera)
                DetectionResult* best_det = nullptr;
                int max_area = 0;
                for (auto& det : detections) {
                    int area = det.bounding_box.width * det.bounding_box.height;
                    if (area > max_area) {
                        max_area = area;
                        best_det = &det;
                    }
                }

                if (!best_det) {
                    std::cout << "No valid detection found" << std::endl;
                    continue;
                }

                float pixel_height = best_det->bounding_box.height;
                
                // Formula: focal_length = (pixel_height * distance) / real_height
                calculated_focal_length = (pixel_height * known_distance) / object_height;

                std::cout << "\n=== Calibration Result ===" << std::endl;
                std::cout << "Detected: " << best_det->class_name << std::endl;
                std::cout << "Pixel height: " << pixel_height << " px" << std::endl;
                std::cout << "Object real height: " << object_height << " m" << std::endl;
                std::cout << "Known distance: " << known_distance << " m" << std::endl;
                std::cout << "Calculated focal length: " << calculated_focal_length << " px" << std::endl;
                std::cout << std::endl;

                // Save to file
                std::string config_file = "camera" + std::to_string(camera_id) + "_calibration.txt";
                std::ofstream out(config_file);
                if (out.is_open()) {
                    out << "# Camera " << camera_id << " Calibration" << std::endl;
                    out << "# Generated by Quick Calibration Tool" << std::endl;
                    out << "focal_length_y=" << calculated_focal_length << std::endl;
                    out << "focal_length_x=" << calculated_focal_length << std::endl;
                    out << "center_x=" << (img_width / 2.0) << std::endl;
                    out << "center_y=" << (img_height / 2.0) << std::endl;
                    out << "image_width=" << img_width << std::endl;
                    out << "image_height=" << img_height << std::endl;
                    out.close();
                    std::cout << "Saved to: " << config_file << std::endl;
                }

                // Automatically update CameraTest.cpp
                std::cout << std::endl;
                std::cout << "Updating CameraTest.cpp with new focal length..." << std::endl;
                
                std::string camera_test_path = "src/object-detection/CameraTest.cpp";
                std::ifstream in_file(camera_test_path);
                std::stringstream buffer;
                buffer << in_file.rdbuf();
                in_file.close();
                
                std::string content = buffer.str();
                
                // Find and replace the focal length lines
                size_t pos1 = content.find("202.5, 0, 640,    // fx, 0, cx");
                size_t pos2 = content.find("0, 202.5, 400,    // 0, fy, cy");
                
                if (pos1 != std::string::npos && pos2 != std::string::npos) {
                    // Replace first occurrence
                    std::string new_line1 = std::to_string(calculated_focal_length) + ", 0, 640,    // fx, 0, cx";
                    content.replace(pos1, strlen("202.5, 0, 640,    // fx, 0, cx"), new_line1);
                    
                    // Find second occurrence again (position changed after first replace)
                    pos2 = content.find("0, 202.5, 400,    // 0, fy, cy");
                    std::string new_line2 = "0, " + std::to_string(calculated_focal_length) + ", 400,    // 0, fy, cy";
                    content.replace(pos2, strlen("0, 202.5, 400,    // 0, fy, cy"), new_line2);
                    
                    // Also update the message
                    size_t pos3 = content.find("Using calibrated focal length: 202.5px");
                    if (pos3 != std::string::npos) {
                        std::string new_msg = "Using calibrated focal length: " + std::to_string(static_cast<int>(calculated_focal_length)) + "px";
                        content.replace(pos3, strlen("Using calibrated focal length: 202.5px"), new_msg);
                    }
                    
                    // Write back
                    std::ofstream out_file(camera_test_path);
                    out_file << content;
                    out_file.close();
                    
                    std::cout << "✓ Updated CameraTest.cpp" << std::endl;
                } else {
                    std::cout << "Warning: Could not find focal length lines in CameraTest.cpp" << std::endl;
                }

                // Automatically recompile
                std::cout << std::endl;
                std::cout << "Recompiling camera_test..." << std::endl;
                int ret = system("cd build && make camera_test -j4 > /dev/null 2>&1");
                if (ret == 0) {
                    std::cout << "✓ Recompilation successful!" << std::endl;
                    std::cout << std::endl;
                    std::cout << "=== Calibration Complete ===" << std::endl;
                    std::cout << "You can now run: build/object-detection/camera_test " << camera_id << std::endl;
                    std::cout << "with accurate distance measurements!" << std::endl;
                } else {
                    std::cout << "✗ Recompilation failed. Please run 'cd build && make camera_test' manually" << std::endl;
                }

                calibrated = true;
            }
        }

        if (!use_simulator) {
            cap.release();
        }
        cv::destroyAllWindows();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
