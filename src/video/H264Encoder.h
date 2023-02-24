#pragma once
#include <string>
#include <vector>

#include <H264Encoder/encoder.hpp>
#include <opencv2/core/core.hpp>

namespace video {
class H264Encoder {
private:
	std::unique_ptr<h264encoder::Encoder> encoder;
	int fps;

public:
	H264Encoder(int fps);

	/**
	 * first frame will set the resolution, subsequent frames must match the resolution
	 */
	std::vector<std::basic_string<uint8_t>> encode_frame(const cv::Mat& frame);
};
} // namespace video