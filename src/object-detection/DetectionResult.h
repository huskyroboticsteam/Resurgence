#pragma once

#include <string>
#include <opencv2/core.hpp>

namespace ObjDet {

/**
 * @brief Structure representing a single detected object in an image.
 * 
 * Contains information about the detected object including its class,
 * bounding box location, and confidence score.
 */
struct DetectionResult {
    int class_id;
    std::string class_name;
    cv::Rect bounding_box;
    float confidence;
    float actual_distance_meters;  // Estimated actual distance in meters (-1.0 = unknown)
    
    DetectionResult(int id, const std::string& name, cv::Rect bbox, float conf, float dist = -1.0f)
        : class_id(id), class_name(name), bounding_box(bbox), confidence(conf), actual_distance_meters(dist) {}
    
    /**
     * @brief Get the center point of the bounding box.
     * @return Center point in pixel coordinates.
     */
    cv::Point2f getCenter() const;
};

} // namespace ObjDet
