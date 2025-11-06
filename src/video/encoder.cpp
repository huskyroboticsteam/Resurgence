#include "encoder.hpp"
#include <stdexcept>
#include <sstream>
#include <iostream>
#include <cstring>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

namespace h264encoder {

class EncoderImpl {
private:
    int in_xres, in_yres, out_xres, out_yres;
    float fps;
    uint32_t bitrate;

    // GStreamer state
    bool gst_initialized = false;
    GstElement* pipeline = nullptr;
    GstElement* appsrc   = nullptr;
    GstElement* appsink  = nullptr;
    guint64 pts = 0;
    guint64 frame_duration_ns = 0;

    // OpenCV input format info
    AVPixelFormat cam_pixel_fmt = AV_PIX_FMT_BGR24; // kept only for API parity

public:
    std::vector<x264_nal_t_simple> nals_simple; // NOTE: pointers inside must not outlive map
    int num_nals = 0;

    EncoderImpl() = default;

    EncoderImpl(int inW, int inH, int outW, int outH, float fps_, int /*rf*/, uint32_t br)
        : in_xres(inW), in_yres(inH), out_xres(outW), out_yres(outH), fps(fps_), bitrate(br)
    {
        gst_init(nullptr, nullptr);

        frame_duration_ns = static_cast<guint64>(1e9 / fps);

        // Build NVENC pipeline
        std::ostringstream ss;
        ss
        << "appsrc name=src is-live=true block=true format=TIME "
           "caps=video/x-raw,format=BGR,width=" << out_xres
        << ",height=" << out_yres << ",framerate=" << (int)fps << "/1 ! "
        << "videoconvert ! "
        << "nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' ! "
        << "nvv4l2h264enc bitrate=" << bitrate
        << " insert-sps-pps=true preset-level=1 iframeinterval=" << (int)(fps * 2)
        << " ! h264parse ! appsink name=sink emit-signals=false sync=false "
           "max-buffers=1 drop=true";

        GError* err = nullptr;
        pipeline = gst_parse_launch(ss.str().c_str(), &err);
        if (!pipeline) {
            std::string msg = err ? err->message : "unknown";
            if (err) g_error_free(err);
            throw std::runtime_error("Failed to create NVENC pipeline: " + msg);
        }

        appsrc  = gst_bin_get_by_name(GST_BIN(pipeline), "src");
        appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
        if (!appsrc || !appsink) {
            throw std::runtime_error("Failed to get appsrc/appsink from pipeline");
        }

        // Helpful properties on appsrc
        g_object_set(G_OBJECT(appsrc),
                     "stream-type", 0,          // GST_APP_STREAM_TYPE_STREAM
                     "is-live", TRUE,
                     "do-timestamp", TRUE,
                     "format", GST_FORMAT_TIME,
                     nullptr);

        gst_element_set_state(pipeline, GST_STATE_PLAYING);
        gst_initialized = true;
        pts = 0;
    }

    virtual ~EncoderImpl() {
        if (pipeline) {
            // Push EOS to flush encoder
            if (appsrc) {
                g_signal_emit_by_name(appsrc, "end-of-stream", nullptr);
            }
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
        }
        if (appsrc)
          gst_object_unref(appsrc);
        if (appsink)
          gst_object_unref(appsink);
    }

    // Encode one BGR frame (contiguous)
    int encode(unsigned char* img) {
        if (!gst_initialized) return -1;

        // If the source frame may be non-contiguous, copy row-by-row instead of a flat memcpy.
        const gsize row_bytes = static_cast<gsize>(in_xres) * 3;
        const gsize size      = static_cast<gsize>(in_yres) * row_bytes;

        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
        if (!buffer) return -1;

        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_WRITE);

        // Flat copy assumes contiguous input. If not guaranteed, replace with row copy.
        std::memcpy(map.data, img, size);

        gst_buffer_unmap(buffer, &map);

        GST_BUFFER_PTS(buffer)      = pts;
        GST_BUFFER_DTS(buffer)      = GST_CLOCK_TIME_NONE;
        GST_BUFFER_DURATION(buffer) = frame_duration_ns;
        pts += frame_duration_ns;

        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);
        if (ret != GST_FLOW_OK) {
            std::cerr << "push-buffer failed: " << ret << std::endl;
            return -1;
        }

        // Pull one encoded sample (non-blocking with timeout)
        GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink), 100000); // 100 ms
        if (!sample) {
            // No output yet (e.g., B-frames or initial delay)
            nals_simple.clear();
            return 0;
        }

        GstBuffer* outbuf = gst_sample_get_buffer(sample);
        GstMapInfo outmap;
        nals_simple.clear();

        if (gst_buffer_map(outbuf, &outmap, GST_MAP_READ)) {
            // WARNING: outmap.data becomes invalid after unmap; do not store pointer long-term
            x264_nal_t_simple nal;
            nal.i_payload = static_cast<int>(outmap.size);
            nal.p_payload = (uint8_t*) outmap.data;
            nals_simple.push_back(nal);
            gst_buffer_unmap(outbuf, &outmap);
        }

        gst_sample_unref(sample);

        // Return the total encoded bytes for this call (single buffer here)
        return nals_simple.empty() ? 0 : nals_simple[0].i_payload;
    }
};

// thin wrapper stays the same
Encoder::Encoder(int inW, int inH, int outW, int outH, float fps, int rf, uint32_t bitrate) {
    impl = std::make_shared<EncoderImpl>(inW, inH, outW, outH, fps, rf, bitrate);
}
int Encoder::encode(unsigned char* img) { return impl->encode(img); }
const std::vector<x264_nal_t_simple>& Encoder::getNals() { return impl->nals_simple; }

} // namespace h264encoder
