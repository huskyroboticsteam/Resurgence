#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

namespace video {
class H264Encoder {
private:
    Encoder encoder;
    bool flag;

public:
    H264Encoder(int width, int height, int fps);

    /**
     * Frame must be the same resolution as passed in the constructor.
    */
    std::vector<std::basic_string<uint8_t>> encode_frame(const cv::Mat& frame);
};
}  // namespace video