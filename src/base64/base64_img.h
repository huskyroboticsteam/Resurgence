#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace base64 {

/**
 * @brief Decode a base64 string that represents an image. Even if images are in grayscale,
 * they will be converted to color.
 *
 * @param b64 The base64-encoded string representing the image.
 * @return cv::Mat A Mat containing the decoded image, in color.
 */
cv::Mat decodeMat(const std::string& b64);

/**
 * @brief Encode an image as a base64 string.
 *
 * @param mat The image to encode.
 * @param format The format string representing how the image should be encoded.
 * See cv::imwrite for a list of supported formats. ".jpg" and ".png" are supported.
 * @return std::string The base64 encoded image.
 */
std::string encodeMat(const cv::Mat& mat, const std::string& format);

} // namespace base64
