#include "H265NVENCEncoder.h"

#include <algorithm>
#include <loguru.hpp>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <cstring>
#include <cinttypes>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

void ensureGStreamerInitialized() {
	static std::once_flag gstInitFlag;
	std::call_once(gstInitFlag, []() {
		int argc = 0;
		char** argv = nullptr;
		gst_init(&argc, &argv);
	});
}

GstClockTime samplePullTimeout(int fps) {
	if (fps <= 0) {
		return GST_MSECOND * 50;
	}
	return std::max<GstClockTime>(GST_MSECOND * 5,
								  GST_SECOND / static_cast<GstClockTime>(fps));
}

std::string buildPipelinePrefix(int width, int height, int fps, const std::string& srcName) {
	std::stringstream pipeline;
	pipeline << "appsrc name=" << srcName
			 << " is-live=true format=time do-timestamp=true block=false "
			 << "caps=video/x-raw,format=BGR,width=" << width << ",height=" << height
			 << ",framerate=" << fps << "/1 ! ";
	pipeline << "queue max-size-buffers=1 leaky=downstream ! ";
	pipeline << "videoconvert ! ";
	return pipeline.str();
}

std::string buildPipelineSuffix(const std::string& sinkName) {
	std::stringstream pipeline;
	pipeline << "h264parse config-interval=1 disable-passthrough=false ! ";
	pipeline << "video/x-h264,stream-format=byte-stream,alignment=au ! ";
	pipeline << "queue max-size-buffers=1 leaky=downstream ! ";
	pipeline << "appsink name=" << sinkName
			 << " caps=\"video/x-h264,stream-format=byte-stream,alignment=au\" "
				"emit-signals=false sync=false drop=true max-buffers=1";
	return pipeline.str();
}

struct EncoderCandidate {
	std::string label;
	std::string pipeline;
	bool hardware = false;
};

void logBusMessages(GstElement* pipeline, const char* context, bool includeStateChanges = false) {
	if (!pipeline) {
		return;
	}

	GstBus* bus = gst_element_get_bus(pipeline);
	if (!bus) {
		return;
	}

	while (true) {
		GstMessage* msg = gst_bus_pop(bus);
		if (!msg) {
			break;
		}

		switch (GST_MESSAGE_TYPE(msg)) {
		case GST_MESSAGE_ERROR: {
			GError* err = nullptr;
			gchar* debug = nullptr;
			gst_message_parse_error(msg, &err, &debug);
			LOG_F(ERROR, "%s: GStreamer error from %s: %s%s%s",
				  context,
				  GST_OBJECT_NAME(msg->src),
				  err ? err->message : "unknown",
				  debug ? " | debug: " : "",
				  debug ? debug : "");
			if (err) {
				g_error_free(err);
			}
			if (debug) {
				g_free(debug);
			}
			break;
		}
		case GST_MESSAGE_WARNING: {
			GError* err = nullptr;
			gchar* debug = nullptr;
			gst_message_parse_warning(msg, &err, &debug);
			LOG_F(WARNING, "%s: GStreamer warning from %s: %s%s%s",
				  context,
				  GST_OBJECT_NAME(msg->src),
				  err ? err->message : "unknown",
				  debug ? " | debug: " : "",
				  debug ? debug : "");
			if (err) {
				g_error_free(err);
			}
			if (debug) {
				g_free(debug);
			}
			break;
		}
		case GST_MESSAGE_STATE_CHANGED: {
			if (includeStateChanges && GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline)) {
				GstState oldState;
				GstState newState;
				GstState pendingState;
				gst_message_parse_state_changed(msg, &oldState, &newState, &pendingState);
				LOG_F(INFO,
					  "%s: pipeline state changed %s -> %s (pending=%s)",
					  context,
					  gst_element_state_get_name(oldState),
					  gst_element_state_get_name(newState),
					  gst_element_state_get_name(pendingState));
			}
			break;
		}
		default:
			break;
		}

		gst_message_unref(msg);
	}

	gst_object_unref(bus);
}

std::vector<EncoderCandidate> buildPipelineCandidates(int width, int height, int fps,
													  const std::string& srcName,
													  const std::string& sinkName) {
	const std::string prefix = buildPipelinePrefix(width, height, fps, srcName);
	const std::string suffix = buildPipelineSuffix(sinkName);
	std::stringstream x264Config;
	x264Config << "x264enc tune=zerolatency speed-preset=ultrafast key-int-max="
			   << std::max(1, fps)
			   << " bframes=0 byte-stream=true aud=true ! ";
	return {
		{"x264enc", prefix + x264Config.str() + suffix, false},
		{"nvh264enc",
		 prefix + "nvh264enc ! video/x-h264,stream-format=byte-stream,alignment=au ! " + suffix,
		 true},
	};
}

void cleanupPipeline(GstElement*& pipeline, GstElement*& appsrc, GstElement*& appsink) {
	if (appsrc) {
		gst_object_unref(appsrc);
		appsrc = nullptr;
	}
	if (appsink) {
		gst_object_unref(appsink);
		appsink = nullptr;
	}
	if (pipeline) {
		gst_element_set_state(pipeline, GST_STATE_NULL);
		gst_object_unref(pipeline);
		pipeline = nullptr;
	}
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

	const std::string srcName = "mc_h264_src";
	const std::string sinkName = "mc_h264_sink";
	const auto candidates = buildPipelineCandidates(width, height, _fps, srcName, sinkName);

	std::string errors;
	for (const auto& candidate : candidates) {
		const char* mode = candidate.hardware ? "hardware" : "software";
		LOG_F(INFO, "Trying %s camera encoder candidate: %s", mode, candidate.label.c_str());
		LOG_F(INFO, "Encoder pipeline candidate (%s): %s",
			  candidate.label.c_str(),
			  candidate.pipeline.c_str());

		GError* parseError = nullptr;
		GstElement* pipeline = gst_parse_launch(candidate.pipeline.c_str(), &parseError);
		if (!pipeline) {
			std::string errorMsg = parseError ? parseError->message : "unknown parse failure";
			LOG_F(WARNING, "Failed to create %s camera encoder pipeline (%s): %s",
				  mode, candidate.label.c_str(), errorMsg.c_str());
			if (parseError) {
				g_error_free(parseError);
			}
			errors += candidate.label + ": " + errorMsg + "; ";
			continue;
		}

		GstElement* appsrc = gst_bin_get_by_name(GST_BIN(pipeline), srcName.c_str());
		GstElement* appsink = gst_bin_get_by_name(GST_BIN(pipeline), sinkName.c_str());
		if (!appsrc || !appsink) {
			LOG_F(WARNING, "Failed to locate appsrc/appsink for %s camera encoder (%s)",
				  mode, candidate.label.c_str());
			cleanupPipeline(pipeline, appsrc, appsink);
			errors += candidate.label + ": missing appsrc/appsink; ";
			continue;
		}

		GstStateChangeReturn stateChange = gst_element_set_state(pipeline, GST_STATE_PLAYING);
		if (stateChange == GST_STATE_CHANGE_FAILURE) {
			logBusMessages(pipeline, "encoder startup failure", true);
			LOG_F(WARNING, "Failed to start %s camera encoder (%s)", mode, candidate.label.c_str());
			cleanupPipeline(pipeline, appsrc, appsink);
			errors += candidate.label + ": unable to start pipeline; ";
			continue;
		}
		logBusMessages(pipeline, "encoder startup", true);

		_pipeline = pipeline;
		_appsrc = appsrc;
		_appsink = appsink;
		_active_encoder_label = candidate.label;
		LOG_F(INFO, "Initialized %s camera encoder: %s (%dx%d@%d)",
			  mode, _active_encoder_label.c_str(), _width, _height, _fps);
		if (!candidate.hardware) {
			LOG_F(WARNING, "Camera stream is using SOFTWARE encoding (%s)", _active_encoder_label.c_str());
		}
		return;
	}

	throw std::runtime_error("Failed to initialize camera encoder pipeline. " + errors);
}

std::vector<std::basic_string<uint8_t>> H265NVENCEncoder::encode_frame(const cv::Mat& frame) {
	std::vector<std::basic_string<uint8_t>> nalUnits;
	if (frame.empty()) {
		return nalUnits;
	}

	if (frame.type() != CV_8UC3) {
		LOG_F(WARNING, "Camera encoder expects CV_8UC3 frame, got type=%d", frame.type());
		return nalUnits;
	}

	if (_init_failed) {
		return nalUnits;
	}

	if (!_pipeline) {
		try {
			initializePipeline(frame.cols, frame.rows);
		} catch (const std::exception& e) {
			LOG_F(ERROR, "Failed to initialize camera encoder pipeline: %s", e.what());
			_init_failed = true;
			return nalUnits;
		}
	}

	if (frame.cols != _width || frame.rows != _height) {
		LOG_F(WARNING,
			  "Camera encoder resolution changed from %dx%d to %dx%d; dropping frame",
			  _width, _height, frame.cols, frame.rows);
		return nalUnits;
	}

	cv::Mat contiguous = frame.isContinuous() ? frame : frame.clone();
	const size_t frameBytes =
		static_cast<size_t>(contiguous.cols * contiguous.rows * contiguous.elemSize());

	GstBuffer* inputBuffer = gst_buffer_new_allocate(nullptr, frameBytes, nullptr);
	if (!inputBuffer) {
		LOG_F(ERROR, "Failed to allocate GstBuffer for camera encoder frame");
		return nalUnits;
	}

	GstMapInfo writeMap;
	if (!gst_buffer_map(inputBuffer, &writeMap, GST_MAP_WRITE)) {
		LOG_F(ERROR, "Failed to map GstBuffer for camera encoder frame");
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
		logBusMessages(_pipeline, "encoder push failure");
		LOG_F(ERROR, "Camera encoder appsrc push failed with status=%d", pushStatus);
		return nalUnits;
	}

	GstSample* sample =
		gst_app_sink_try_pull_sample(GST_APP_SINK(_appsink), samplePullTimeout(_fps));
	if (!sample) {
		_consecutive_empty_pulls++;
		logBusMessages(_pipeline, "encoder empty pull");
		if ((_consecutive_empty_pulls % 120) == 0) {
			LOG_F(WARNING,
				  "Camera encoder (%s) produced no output for %u consecutive frames",
				  _active_encoder_label.c_str(), _consecutive_empty_pulls);
		}
		return nalUnits;
	}
	_consecutive_empty_pulls = 0;

	GstBuffer* outputBuffer = gst_sample_get_buffer(sample);
	GstMapInfo readMap;
	if (outputBuffer && gst_buffer_map(outputBuffer, &readMap, GST_MAP_READ)) {
		if (_frame_counter <= 5) {
			LOG_F(INFO,
				  "Camera encoder (%s) produced sample of %zu bytes on frame %" PRIu64,
				  _active_encoder_label.c_str(),
				  static_cast<size_t>(readMap.size),
				  _frame_counter);
		}
		nalUnits.emplace_back(readMap.data, readMap.data + readMap.size);
		gst_buffer_unmap(outputBuffer, &readMap);
	}

	gst_sample_unref(sample);
	return nalUnits;
}

} // namespace video
