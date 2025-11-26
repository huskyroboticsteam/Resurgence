#include "H264PassthroughSource.h"

#include <atomic>
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

std::string buildPipelineString(const cam::CameraStreamProperties& props, const std::string& sinkName) {
	std::stringstream pipeline;
	pipeline << "v4l2src device=/dev/video" << props.cameraId << " ! ";
	pipeline << props.format << ",width=" << props.width << ",height=" << props.height
			 << ",framerate=" << props.framerate << "/1 ! ";
	pipeline << "h264parse config-interval=1 disable-passthrough=false ! ";
	pipeline << "queue leaky=2 ! ";
	pipeline << "appsink name=" << sinkName
			 << " caps=\"video/x-h264,stream-format=byte-stream,alignment=au\" "
				"emit-signals=false sync=false drop=true max-buffers=1";
	return pipeline.str();
}

} // namespace

namespace cam {

H264PassthroughSource::H264PassthroughSource(const CameraStreamProperties& props)
	: _framerate(props.framerate) {
	initializePipeline(props);
}

H264PassthroughSource::~H264PassthroughSource() {
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

void H264PassthroughSource::initializePipeline(const CameraStreamProperties& props) {
	ensureGStreamerInitialized();

	static std::atomic<uint32_t> sinkCounter{0};
	std::string sinkName = "mcappsink_" + std::to_string(sinkCounter.fetch_add(1));

	GError* parseError = nullptr;
	std::string pipelineStr = buildPipelineString(props, sinkName);
	_pipeline = gst_parse_launch(pipelineStr.c_str(), &parseError);
	if (!_pipeline) {
		std::string errorMsg = "Failed to create GStreamer pipeline: ";
		if (parseError) {
			errorMsg += parseError->message;
			g_error_free(parseError);
		}
		throw std::runtime_error(errorMsg);
	}

	_appsink = gst_bin_get_by_name(GST_BIN(_pipeline), sinkName.c_str());
	if (!_appsink) {
		gst_object_unref(_pipeline);
		_pipeline = nullptr;
		throw std::runtime_error("Failed to locate appsink in GStreamer pipeline");
	}

	if (gst_element_set_state(_pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
		gst_object_unref(_appsink);
		gst_object_unref(_pipeline);
		_appsink = nullptr;
		_pipeline = nullptr;
		throw std::runtime_error("Unable to start GStreamer pipeline");
	}
}

bool H264PassthroughSource::next(std::vector<std::basic_string<uint8_t>>& nalUnits, uint32_t& frameNum) {
	nalUnits.clear();
	if (!_appsink) {
		return false;
	}

	GstSample* sample = gst_app_sink_try_pull_sample(
		GST_APP_SINK(_appsink),
		_framerate > 0 ? GST_SECOND / static_cast<GstClockTime>(_framerate) : GST_MSECOND * 50);
	if (!sample) {
		return false;
	}

	GstBuffer* buffer = gst_sample_get_buffer(sample);
	GstMapInfo map;
	bool success = false;
	if (buffer && gst_buffer_map(buffer, &map, GST_MAP_READ)) {
		std::basic_string<uint8_t> nal(map.data, map.data + map.size);
		nalUnits.push_back(std::move(nal));
		gst_buffer_unmap(buffer, &map);
		frameNum = ++_frameCounter;
		success = true;
	}

	gst_sample_unref(sample);
	return success;
}

} // namespace cam
