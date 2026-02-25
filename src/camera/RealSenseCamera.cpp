#include "RealSenseCamera.h"

#include <algorithm>
#include <loguru.hpp>
#include <opencv2/imgproc.hpp>

namespace cam {

RealSenseCamera::RealSenseCamera(const std::string& serial_number)
    : serial_number_(serial_number) {
}

RealSenseCamera::~RealSenseCamera() {
    stop();
}

bool RealSenseCamera::start(int color_width, int color_height, int fps) {
    if (running_) {
        LOG_F(WARNING, "RealSense camera already running");
        return true;
    }
    
    try {
        rs2::config cfg;
        
        // If serial number specified, use it
        if (!serial_number_.empty()) {
            cfg.enable_device(serial_number_);
        }
        
        // Enable color stream
        cfg.enable_stream(RS2_STREAM_COLOR, color_width, color_height, 
                          RS2_FORMAT_BGR8, fps);
        
        // Enable depth stream (same resolution for alignment)
        cfg.enable_stream(RS2_STREAM_DEPTH, color_width, color_height,
                          RS2_FORMAT_Z16, fps);
        
        // Start pipeline
        profile_ = pipeline_.start(cfg);
        
        // Get depth scale from the depth sensor
        auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();
        depth_scale_ = depth_sensor.get_depth_scale();
        LOG_F(INFO, "RealSense depth scale: %f m/unit", depth_scale_);
        
        // Get intrinsic parameters from color stream
        auto color_stream = profile_.get_stream(RS2_STREAM_COLOR)
                                    .as<rs2::video_stream_profile>();
        auto intrinsics = color_stream.get_intrinsics();
        
        // Convert to OpenCV format
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            intrinsics.fx, 0, intrinsics.ppx,
            0, intrinsics.fy, intrinsics.ppy,
            0, 0, 1);
        
        // D455 uses Brown-Conrady distortion model (5 coefficients)
        cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) <<
            intrinsics.coeffs[0], intrinsics.coeffs[1],
            intrinsics.coeffs[2], intrinsics.coeffs[3],
            intrinsics.coeffs[4]);
        
        intrinsic_params_ = CameraParams(camera_matrix, dist_coeffs,
                                          cv::Size(color_width, color_height));
        has_intrinsics_ = true;
        
        LOG_F(INFO, "RealSense D455 intrinsics: fx=%f, fy=%f, cx=%f, cy=%f",
              intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy);
        
        running_ = true;
        capture_thread_ = std::make_unique<std::thread>(&RealSenseCamera::captureLoop, this);
        
        LOG_F(INFO, "RealSense camera started: %dx%d @ %d fps", 
              color_width, color_height, fps);
        return true;
        
    } catch (const rs2::error& e) {
        LOG_F(ERROR, "RealSense error: %s (%s)", e.what(), 
              e.get_failed_function().c_str());
        return false;
    } catch (const std::exception& e) {
        LOG_F(ERROR, "Failed to start RealSense: %s", e.what());
        return false;
    }
}

void RealSenseCamera::stop() {
    if (!running_) return;
    
    running_ = false;
    
    if (capture_thread_ && capture_thread_->joinable()) {
        capture_thread_->join();
    }
    capture_thread_.reset();
    
    try {
        pipeline_.stop();
    } catch (...) {
        // Ignore errors during shutdown
    }
    
    LOG_F(INFO, "RealSense camera stopped");
}

bool RealSenseCamera::isRunning() const {
    return running_;
}

bool RealSenseCamera::hasNext(uint32_t old_frame_num) const {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    return frame_num_ > old_frame_num;
}

void RealSenseCamera::captureLoop() {
    LOG_F(INFO, "RealSense capture thread started");
    
    while (running_) {
        try {
            // Wait for frames with timeout
            rs2::frameset frameset;
            if (!pipeline_.poll_for_frames(&frameset)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            
            // Align depth to color (standard processing like realsense-viewer)
            auto aligned = align_to_color_.process(frameset);
            
            auto color_frame = aligned.get_color_frame();
            auto depth_frame = aligned.get_depth_frame();
            
            if (!color_frame || !depth_frame) {
                continue;
            }
            
            // Apply SDK colorizer to depth (same as realsense-viewer)
            auto colorized_depth = colorizer_.colorize(depth_frame);
            
            // Convert to OpenCV Mat
            cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()),
                          CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth(cv::Size(depth_frame.get_width(), depth_frame.get_height()),
                          CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat depth_color(cv::Size(colorized_depth.get_width(), colorized_depth.get_height()),
                                CV_8UC3, (void*)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
            
            // Update shared frame data
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                color.copyTo(color_frame_);
                depth.copyTo(depth_frame_);
                depth_color.copyTo(depth_colorized_);
                frame_num_++;
                frame_time_ = robot::types::dataclock::now();
            }
            
        } catch (const rs2::error& e) {
            LOG_F(ERROR, "RealSense capture error: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    LOG_F(INFO, "RealSense capture thread stopped");
}

bool RealSenseCamera::getColorFrame(cv::Mat& color_frame, uint32_t& frame_num,
                                     robot::types::datatime_t& timestamp) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (color_frame_.empty()) {
        return false;
    }
    color_frame_.copyTo(color_frame);
    frame_num = frame_num_;
    timestamp = frame_time_;
    return true;
}

bool RealSenseCamera::getDepthFrame(cv::Mat& depth_frame, uint32_t& frame_num,
                                     robot::types::datatime_t& timestamp) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (depth_frame_.empty()) {
        return false;
    }
    depth_frame_.copyTo(depth_frame);
    frame_num = frame_num_;
    timestamp = frame_time_;
    return true;
}

bool RealSenseCamera::getFrames(cv::Mat& color_frame, cv::Mat& depth_frame,
                                 uint32_t& frame_num, robot::types::datatime_t& timestamp) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (color_frame_.empty() || depth_frame_.empty()) {
        return false;
    }
    color_frame_.copyTo(color_frame);
    depth_frame_.copyTo(depth_frame);
    frame_num = frame_num_;
    timestamp = frame_time_;
    return true;
}

bool RealSenseCamera::getColorizedDepth(cv::Mat& depth_colorized) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (depth_colorized_.empty()) {
        return false;
    }
    depth_colorized_.copyTo(depth_colorized);
    return true;
}

float RealSenseCamera::getDistanceAtPixel(int x, int y) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (depth_frame_.empty() || x < 0 || y < 0 ||
        x >= depth_frame_.cols || y >= depth_frame_.rows) {
        return -1.0f;
    }
    uint16_t raw_depth = depth_frame_.at<uint16_t>(y, x);
    if (raw_depth == 0) {
        return -1.0f;  // Invalid depth reading
    }
    return static_cast<float>(raw_depth) * depth_scale_;
}

float RealSenseCamera::getDistanceToObject(const cv::Rect& bbox) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (depth_frame_.empty()) {
        return -1.0f;
    }
    
    // Sample from center region (inner 50% of bbox)
    int cx = bbox.x + bbox.width / 2;
    int cy = bbox.y + bbox.height / 2;
    int sample_w = bbox.width / 4;
    int sample_h = bbox.height / 4;
    
    int x1 = std::max(0, cx - sample_w);
    int y1 = std::max(0, cy - sample_h);
    int x2 = std::min(depth_frame_.cols - 1, cx + sample_w);
    int y2 = std::min(depth_frame_.rows - 1, cy + sample_h);
    
    // Collect valid depth values
    std::vector<float> depths;
    depths.reserve((x2 - x1 + 1) * (y2 - y1 + 1));
    
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            uint16_t raw = depth_frame_.at<uint16_t>(y, x);
            if (raw > 0) {
                depths.push_back(static_cast<float>(raw) * depth_scale_);
            }
        }
    }
    
    if (depths.empty()) {
        return -1.0f;
    }
    
    // Return median for robustness
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    return depths[depths.size() / 2];
}

CameraParams RealSenseCamera::getIntrinsicParams() const {
    return intrinsic_params_;
}

bool RealSenseCamera::hasIntrinsicParams() const {
    return has_intrinsics_;
}

float RealSenseCamera::getDepthScale() const {
    return depth_scale_;
}

// DepthCameraFrame implementation
float DepthCameraFrame::getDistanceToRect(const cv::Rect& bbox) const {
    if (depth.empty()) {
        return -1.0f;
    }
    
    int cx = bbox.x + bbox.width / 2;
    int cy = bbox.y + bbox.height / 2;
    int sample_w = bbox.width / 4;
    int sample_h = bbox.height / 4;
    
    int x1 = std::max(0, cx - sample_w);
    int y1 = std::max(0, cy - sample_h);
    int x2 = std::min(depth.cols - 1, cx + sample_w);
    int y2 = std::min(depth.rows - 1, cy + sample_h);
    
    std::vector<float> depths;
    depths.reserve((x2 - x1 + 1) * (y2 - y1 + 1));
    
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            uint16_t raw = depth.at<uint16_t>(y, x);
            if (raw > 0) {
                depths.push_back(static_cast<float>(raw) * depth_scale);
            }
        }
    }
    
    if (depths.empty()) {
        return -1.0f;
    }
    
    std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
    return depths[depths.size() / 2];
}

} // namespace cam
