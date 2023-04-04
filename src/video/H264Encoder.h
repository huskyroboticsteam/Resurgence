#pragma once
#include "encoder.hpp"

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace video {
/**
 * This class is a wrapper for the h264encoder library
 */
class H264Encoder {
public:
	/**
	 * Uses the h264encoder library to encode webcam frames into encoded video data
	 */
	H264Encoder(int fps);
	/**
	 * First frame will set the resolution and subsequent frames must match the resolution
	 * @returns A vector of video data units representing the encoded frame
	 */
	std::vector<std::basic_string<uint8_t>> encode_frame(const cv::Mat& frame);

private:
	// A pointer to the h264encoder object that will process frames
	std::unique_ptr<h264encoder::Encoder> encoder;
	// The frame rate of the video being encoded
	int fps;
};
} // namespace video