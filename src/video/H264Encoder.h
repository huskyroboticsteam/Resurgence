#pragma once
#include "encoder.hpp"

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace video {
/**
 * This class encodes OpenCV frames into encoded H264 data that can be sent to an H264 decoder,
 * like JMuxer.
 */
class H264Encoder {
public:
	/**
	 * Uses the h264encoder library to encode webcam frames into encoded video data
	 * @param fps The framerate of the input video/frames to be encoded.
	 * @param rf The RF factor for the encoder.  Max 51, Min 1.  Higher values mean more
	 * compression and smaller encoded frame sizes, but they also lead to lower quality.  This
	 * value shouldn't be below 21 as values below this have essentially the same quality while
	 * having much larger encoded frame sizes.
	 */
	//H264Encoder(int fps, int rf);
  H264Encoder(int fps, int rf, uint32_t bitrate)
	/**
	 * First frame will set the resolution and subsequent frames must match the resolution
	 * @param frame A reference to an OpenCV 2D Matrix representing the frame to be encoded.
	 * @returns A vector of video data units representing the encoded frame
	 */
	std::vector<std::basic_string<uint8_t>> encode_frame(const cv::Mat& frame);

private:
	// A pointer to the h264encoder object that will process frames
	std::unique_ptr<h264encoder::Encoder> encoder;
	// The frame rate of the video being encoded
	int fps;
	// the rf constant for rate control compression
	int rf;
  uint32_t bitrate;
};
} // namespace video
