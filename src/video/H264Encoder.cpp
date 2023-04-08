#include "H264Encoder.h"

namespace video {
H264Encoder::H264Encoder(int fps) : fps(fps) {}

std::vector<std::basic_string<uint8_t>> H264Encoder::encode_frame(const cv::Mat& frame) {
	if (!encoder) {
		auto size = frame.size();
		encoder = std::make_unique<h264encoder::Encoder>(size.width, size.height, size.width,
														 size.height, fps);
	}
	std::vector<std::basic_string<uint8_t>> video_data_units;
	auto frame_size = encoder->encode(frame.data);
	for (auto video_data : encoder->getNals()) {
		std::basic_string<uint8_t> data(video_data.p_payload, video_data.i_payload);
		video_data_units.push_back(data);
	}
	return video_data_units;
}
} // namespace video