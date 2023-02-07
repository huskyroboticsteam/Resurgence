#include "H264Encoder.h"

namespace video{
H264Encoder::H264Encoder(int fps) : fps(fps), flag(true) {
    
}

std::vector<std::basic_string<uint8_t>> H264Encoder::encode_frame(const cv::Mat& frame) {
    if (!encoder) {
        auto size = frame.size();
        encoder = std::make_unique<Encoder>(size.width, size.height, size.width, size.height, fps);
    }
    std::vector<std::basic_string<uint8_t>> nals;
    auto frame_size = encoder->encode(frame.data, &flag);
    for (auto i = 0; i  < encoder->num_nals; i++) {
        std::basic_string<uint8_t> data = std::basic_string<uint8_t>(encoder->nals[i].p_payload, encoder->nals[i].i_payload);
        nals.push_back(data);        
    }
    return nals;
}
}