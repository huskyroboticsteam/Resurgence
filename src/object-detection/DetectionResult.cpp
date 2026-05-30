#include "DetectionResult.h"

namespace ObjDet {

cv::Point2f DetectionResult::getCenter() const {
    return cv::Point2f(
        bounding_box.x + bounding_box.width / 2.0f,
        bounding_box.y + bounding_box.height / 2.0f
    );
}

} // namespace ObjDet
