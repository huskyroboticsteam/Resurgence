#pragma once

#include "../world_interface/data.h"
#include "CameraParams.h"

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>

/**
 * @namespace cam
 * @brief Namespace for camera access system.
 */
namespace cam {

/**
 * @brief RealSense D455 camera wrapper for RGB + Depth streaming.
 * 
 * This class provides access to Intel RealSense D455 camera, exposing both
 * RGB color stream and aligned depth stream. The depth data can be used for
 * accurate distance measurement to detected objects.
 * 
 * Key features:
 * - Simultaneous RGB and depth streaming
 * - Depth aligned to color frame for easy pixel-to-depth mapping
 * - Thread-safe frame access
 * - Automatic intrinsic parameter extraction from camera firmware
 */
class RealSenseCamera {
public:
    /**
     * @brief Construct a RealSense camera interface.
     * @param serial_number Optional serial number to select specific camera.
     *                      If empty, the first available D455 will be used.
     */
    explicit RealSenseCamera(const std::string& serial_number = "");
    
    ~RealSenseCamera();
    
    // Disable copy
    RealSenseCamera(const RealSenseCamera&) = delete;
    RealSenseCamera& operator=(const RealSenseCamera&) = delete;
    
    /**
     * @brief Start the camera streams.
     * @param color_width Width of color stream (default 640)
     * @param color_height Height of color stream (default 480)
     * @param fps Frames per second (default 30)
     * @return true if started successfully
     */
    bool start(int color_width = 640, int color_height = 480, int fps = 30);
    
    /**
     * @brief Stop the camera streams.
     */
    void stop();
    
    /**
     * @brief Check if camera is running.
     */
    bool isRunning() const;
    
    /**
     * @brief Check if a new frame is available.
     * @param old_frame_num The last frame number the caller has seen.
     * @return true if a newer frame is available.
     */
    bool hasNext(uint32_t old_frame_num) const;
    
    /**
     * @brief Get the latest color frame.
     * @param[out] color_frame The color image (BGR format)
     * @param[out] frame_num The frame number
     * @param[out] timestamp The timestamp when frame was captured
     * @return true if frame was retrieved successfully
     */
    bool getColorFrame(cv::Mat& color_frame, uint32_t& frame_num, 
                       robot::types::datatime_t& timestamp);
    
    /**
     * @brief Get the latest depth frame (aligned to color).
     * @param[out] depth_frame The depth image (16-bit unsigned, in millimeters)
     * @param[out] frame_num The frame number
     * @param[out] timestamp The timestamp when frame was captured
     * @return true if frame was retrieved successfully
     */
    bool getDepthFrame(cv::Mat& depth_frame, uint32_t& frame_num,
                       robot::types::datatime_t& timestamp);
    
    /**
     * @brief Get both color and depth frames.
     * @param[out] color_frame The color image (BGR format)
     * @param[out] depth_frame The depth image (16-bit unsigned, in millimeters)
     * @param[out] frame_num The frame number
     * @param[out] timestamp The timestamp when frame was captured
     * @return true if frames were retrieved successfully
     */
    bool getFrames(cv::Mat& color_frame, cv::Mat& depth_frame,
                   uint32_t& frame_num, robot::types::datatime_t& timestamp);
    
    /**
     * @brief Get colorized depth image (using SDK's built-in colorizer).
     * 
     * This produces the same visualization as realsense-viewer.
     * @param[out] depth_colorized The colorized depth image (BGR format)
     * @return true if available
     */
    bool getColorizedDepth(cv::Mat& depth_colorized);
    
    /**
     * @brief Get the distance at a specific pixel (in meters).
     * @param x X coordinate in color frame
     * @param y Y coordinate in color frame
     * @return Distance in meters, or -1 if invalid
     */
    float getDistanceAtPixel(int x, int y);
    
    /**
     * @brief Get the distance to the center of a bounding box (in meters).
     * @param bbox Bounding box in format cv::Rect(x, y, width, height)
     * @return Distance in meters (median of valid depths in center region), or -1 if invalid
     */
    float getDistanceToObject(const cv::Rect& bbox);
    
    /**
     * @brief Get camera intrinsic parameters (from color stream).
     * @return CameraParams object with intrinsic matrix and distortion coefficients
     */
    CameraParams getIntrinsicParams() const;
    
    /**
     * @brief Check if intrinsic parameters are available.
     */
    bool hasIntrinsicParams() const;
    
    /**
     * @brief Get the depth scale (conversion factor from raw depth to meters).
     * @return Depth scale factor (multiply raw depth value by this to get meters)
     */
    float getDepthScale() const;

private:
    void captureLoop();
    
    rs2::pipeline pipeline_;
    rs2::pipeline_profile profile_;
    rs2::align align_to_color_{RS2_STREAM_COLOR};
    rs2::colorizer colorizer_;  // Built-in depth colorizer (same as realsense-viewer)
    
    std::string serial_number_;
    bool running_ = false;
    float depth_scale_ = 0.001f;  // Default: 1mm per unit
    
    // Frame data (protected by mutex)
    cv::Mat color_frame_;
    cv::Mat depth_frame_;
    cv::Mat depth_colorized_;  // Colorized depth from SDK
    uint32_t frame_num_ = 0;
    robot::types::datatime_t frame_time_;
    mutable std::mutex frame_mutex_;
    
    // Intrinsic parameters
    CameraParams intrinsic_params_;
    bool has_intrinsics_ = false;
    
    // Capture thread
    std::unique_ptr<std::thread> capture_thread_;
};

/**
 * @brief Extended CameraFrame that includes depth data.
 * 
 * This struct extends the basic CameraFrame with depth information
 * for use with RealSense cameras.
 */
struct DepthCameraFrame {
    cv::Mat color;          ///< Color image (BGR)
    cv::Mat depth;          ///< Depth image (16-bit unsigned, millimeters)
    uint32_t frame_num;     ///< Frame number
    float depth_scale;      ///< Conversion factor: meters = raw_value * depth_scale
    
    /**
     * @brief Get distance at a pixel location.
     * @param x X coordinate
     * @param y Y coordinate
     * @return Distance in meters, or -1 if invalid
     */
    float getDistanceAt(int x, int y) const {
        if (depth.empty() || x < 0 || y < 0 || 
            x >= depth.cols || y >= depth.rows) {
            return -1.0f;
        }
        uint16_t raw_depth = depth.at<uint16_t>(y, x);
        if (raw_depth == 0) {
            return -1.0f;  // Invalid depth
        }
        return static_cast<float>(raw_depth) * depth_scale;
    }
    
    /**
     * @brief Get distance to the center of a bounding box.
     * 
     * Uses median filtering in the center region for robustness.
     * @param bbox Bounding box
     * @return Distance in meters, or -1 if invalid
     */
    float getDistanceToRect(const cv::Rect& bbox) const;
};

} // namespace cam
