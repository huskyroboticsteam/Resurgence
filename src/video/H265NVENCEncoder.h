#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

typedef struct _GstElement GstElement;

namespace video {

class H265NVENCEncoder {
public:
	explicit H265NVENCEncoder(int fps);
	~H265NVENCEncoder();

	H265NVENCEncoder(const H265NVENCEncoder&) = delete;
	H265NVENCEncoder& operator=(const H265NVENCEncoder&) = delete;

	std::vector<std::basic_string<uint8_t>> encode_frame(const cv::Mat& frame);

private:
	void initializePipeline(int width, int height);

	int _fps;
	int _width = 0;
	int _height = 0;
	uint64_t _frame_counter = 0;
	GstElement* _pipeline = nullptr;
	GstElement* _appsrc = nullptr;
	GstElement* _appsink = nullptr;
};

} // namespace video
