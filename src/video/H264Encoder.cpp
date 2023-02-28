#include "H264Encoder.h"

namespace video {
H264Encoder::H264Encoder(int fps) : fps(fps) {}

std::vector<std::basic_string<uint8_t>> H264Encoder::encode_frame(const cv::Mat& frame) {
	if (!encoder) {
		auto size = frame.size();
		encoder = std::make_unique<h264encoder::Encoder>(size.width, size.height, size.width,
														 size.height, fps);
	}
	std::vector<std::basic_string<uint8_t>> nals;
	auto frame_size = encoder->encode(frame.data);
	for (auto nal : encoder->nals) {
		std::basic_string<uint8_t> data(nal.p_payload, nal.i_payload);
		nals.push_back(data);
	}
	return nals;
}
} // namespace video