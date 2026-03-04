#include "H265NVENCEncoder.h"

#include <algorithm>
#include <loguru.hpp>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <cstring>
#include <mutex>
#include <sstream>
#include <stdexcept>

namespace {

void ensureGStreamerInitialized() {
	static std::once_flag gstInitFlag;
	std::call_once(gstInitFlag, []() {
		int argc = 0;
		char** argv = nullptr;
		gst_init(&argc, &argv);
	});
}

std::string buildPipelineString(int width, int height, int fps, const std::string& srcName,
								const std::string& sinkName) {
	std::stringstream pipeline;
	pipeline << "appsrc name=" << srcName
			 << " is-live=true format=time do-timestamp=true block=false "
			 << "caps=video/x-raw,format=BGR,width=" << width << ",height=" << height
			 << ",framerate=" << fps << "/1 ! ";
	pipeline << "queue max-size-buffers=1 leaky=downstream ! ";
	pipeline << "videoconvert ! ";
	pipeline << "nvh265enc ! ";
	pipeline << "h265parse config-interval=1 disable-passthrough=false ! ";
	pipeline << "queue max-size-buffers=1 leaky=downstream ! ";
	pipeline << "appsink name=" << sinkName
			 << " caps=\"video/x-h265,stream-format=byte-stream,alignment=au\" "
				"emit-signals=false sync=false drop=true max-buffers=1";
	return pipeline.str();
}

} // namespace

namespace video {

H265NVENCEncoder::H265NVENCEncoder(int fps) : _fps(std::max(1, fps)) {}

H265NVENCEncoder::~H265NVENCEncoder() {
	if (_appsrc) {
		gst_object_unref(_appsrc);
		_appsrc = nullptr;
	}
	if (_appsink) {
		gst_object_unref(_appsink);
		_appsink = nullptr;
	}
	if (_pipeline) {
		gst_element_set_state(_pipeline, GST_STATE_NULL);
		gst_object_unref(_pipeline);
		_pipeline = nullptr;
	}
}

void H265NVENCEncoder::initializePipeline(int width, int height) {
	ensureGStreamerInitialized();
	_width = width;
	_height = height;

	std::string srcName = "mc_h265_src";
	std::string sinkName = "mc_h265_sink";
	std::string pipelineStr = buildPipelineString(width, height, _fps, srcName, sinkName);

	GError* parseError = nullptr;
	_pipeline = gst_parse_launch(pipelineStr.c_str(), &parseError);
	if (!_pipeline) {
		std::string errorMsg = "Failed to create H265 NVENC pipeline: ";
		if (parseError) {
			errorMsg += parseError->message;
			g_error_free(parseError);
		}
		throw std::runtime_error(errorMsg);
	}

	_appsrc = gst_bin_get_by_name(GST_BIN(_pipeline), srcName.c_str());
	_appsink = gst_bin_get_by_name(GST_BIN(_pipeline), sinkName.c_str());
	if (!_appsrc || !_appsink) {
		if (_appsrc) {
			gst_object_unref(_appsrc);
			_appsrc = nullptr;
		}
		if (_appsink) {
			gst_object_unref(_appsink);
			_appsink = nullptr;
		}
		gst_object_unref(_pipeline);
		_pipeline = nullptr;
		throw std::runtime_error("Failed to locate appsrc/appsink for H265 NVENC pipeline");
	}

	if (gst_element_set_state(_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
		gst_object_unref(_appsrc);
		gst_object_unref(_appsink);
		gst_object_unref(_pipeline);
		_appsrc = nullptr;
		_appsink = nullptr;
		_pipeline = nullptr;
		throw std::runtime_error("Unable to start H265 NVENC pipeline");
	}
}

std::vector<std::basic_string<uint8_t>> H265NVENCEncoder::encode_frame(const cv::Mat& frame) {
	std::vector<std::basic_string<uint8_t>> nalUnits;
	if (frame.empty()) {
		return nalUnits;
	}

	if (frame.type() != CV_8UC3) {
		LOG_F(WARNING, "H265 NVENC encoder expects CV_8UC3 frame, got type=%d", frame.type());
		return nalUnits;
	}

	if (!_pipeline) {
		try {
			initializePipeline(frame.cols, frame.rows);
		} catch (const std::exception& e) {
			LOG_F(ERROR, "Failed to initialize H265 NVENC encoder pipeline: %s", e.what());
			return nalUnits;
		}
	}

	if (frame.cols != _width || frame.rows != _height) {
		LOG_F(WARNING,
			  "H265 NVENC encoder resolution changed from %dx%d to %dx%d; dropping frame",
			  _width, _height, frame.cols, frame.rows);
		return nalUnits;
	}

	cv::Mat contiguous = frame.isContinuous() ? frame : frame.clone();
	const size_t frameBytes =
		static_cast<size_t>(contiguous.cols * contiguous.rows * contiguous.elemSize());

	GstBuffer* inputBuffer = gst_buffer_new_allocate(nullptr, frameBytes, nullptr);
	if (!inputBuffer) {
		LOG_F(ERROR, "Failed to allocate GstBuffer for H265 NVENC frame");
		return nalUnits;
	}

	GstMapInfo writeMap;
	if (!gst_buffer_map(inputBuffer, &writeMap, GST_MAP_WRITE)) {
		LOG_F(ERROR, "Failed to map GstBuffer for H265 NVENC frame");
		gst_buffer_unref(inputBuffer);
		return nalUnits;
	}
	std::memcpy(writeMap.data, contiguous.data, frameBytes);
	gst_buffer_unmap(inputBuffer, &writeMap);

	GstClockTime frameDuration = gst_util_uint64_scale_int(1, GST_SECOND, _fps);
	GST_BUFFER_PTS(inputBuffer) = _frame_counter * frameDuration;
	GST_BUFFER_DTS(inputBuffer) = GST_BUFFER_PTS(inputBuffer);
	GST_BUFFER_DURATION(inputBuffer) = frameDuration;
	_frame_counter++;

	GstFlowReturn pushStatus = gst_app_src_push_buffer(GST_APP_SRC(_appsrc), inputBuffer);
	if (pushStatus != GST_FLOW_OK) {
		LOG_F(ERROR, "H265 NVENC appsrc push failed with status=%d", pushStatus);
		return nalUnits;
	}

	GstSample* sample = gst_app_sink_try_pull_sample(
		GST_APP_SINK(_appsink), frameDuration + (10 * GST_MSECOND));
	if (!sample) {
		return nalUnits;
	}

	GstBuffer* outputBuffer = gst_sample_get_buffer(sample);
	GstMapInfo readMap;
	if (outputBuffer && gst_buffer_map(outputBuffer, &readMap, GST_MAP_READ)) {
		nalUnits.emplace_back(readMap.data, readMap.data + readMap.size);
		gst_buffer_unmap(outputBuffer, &readMap);
	}

	gst_sample_unref(sample);
	return nalUnits;
}

} // namespace video
