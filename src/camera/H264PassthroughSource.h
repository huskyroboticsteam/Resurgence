#pragma once

#include "CameraStreamProperties.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

typedef struct _GstElement GstElement;

namespace cam {

class H264PassthroughSource {
public:
	explicit H264PassthroughSource(const CameraStreamProperties& props);
	~H264PassthroughSource();

	bool next(std::vector<std::basic_string<uint8_t>>& nalUnits, uint32_t& frameNum);

private:
	void initializePipeline(const CameraStreamProperties& props);

	GstElement* _pipeline = nullptr;
	GstElement* _appsink = nullptr;
	uint32_t _frameCounter = 0;
	int _framerate = 30;
};

} // namespace cam
