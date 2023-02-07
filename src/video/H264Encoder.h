#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
// need Encoder include

class Encoder {  // delete once Encoder is included

};

namespace video {
class H264Encoder {
private:
    std::unique_ptr<Encoder> encoder;
    bool flag;
    int fps;

public:
    H264Encoder(int fps);

    /**
     * Frame must be the same resolution as passed in the constructor.
    */
    std::vector<std::basic_string<uint8_t>> encode_frame(const cv::Mat& frame);
};
}  // namespace video