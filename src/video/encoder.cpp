#include "encoder.hpp"

#include <iostream>

extern "C" {
#include <x264.h>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace h264encoder {

class EncoderImpl {
private:
	int in_xres, in_yres, out_xres, out_yres;
	int framecounter;
	int nheader;
	x264_t* enc;
	x264_param_t prms;
	x264_picture_t pic_in, pic_out;
	x264_nal_t* nals;

	struct SwsContext* sws;
	AVFrame pic_raw; /* used for our "raw" input container */
	AVPixelFormat cam_pixel_fmt = AV_PIX_FMT_BGR24;

public:
	std::vector<x264_nal_t_simple> nals_simple;
	int num_nals;
	virtual ~EncoderImpl() {}
	EncoderImpl(){};

	// Setup of the encoder instance
	EncoderImpl(int inW, int inH, int outW, int outH, float fps, int rf)
		: in_xres(inW), in_yres(inH), out_xres(outW), out_yres(outH) {

		framecounter = 0;
		x264_param_default_preset(&prms, "ultrafast", "zerolatency,fastdecode");
		x264_param_apply_profile(&prms, "baseline");
		prms.i_width = out_xres;
		prms.i_height = out_yres;
		prms.i_fps_num = fps;
		prms.i_fps_den = 1;
		prms.rc.i_qp_constant = 51; // Quantization parameter, deprecated compression.
									// max value of 51.  Higher value means more compression.

		prms.rc.i_rc_method = X264_RC_CRF; // defines the rate control method as RF
		prms.rc.f_rf_constant = rf; // Rate contorl factor, higher means more compression (max
									// 51). this value shouldn't be below 21 as quality is
									// essentially the same as the input.
		prms.rc.f_rf_constant_max = 25; // the maximum quality, will not go below this value

		prms.i_csp = X264_CSP_I420;
		enc = x264_encoder_open(&prms);

		x264_encoder_headers(enc, &nals, &nheader);

		// Initialize X264 Picture
		x264_picture_alloc(&pic_in, X264_CSP_I420, out_xres, out_yres);

		// Color conversion setup
		sws = sws_getContext(
			in_xres, in_yres, cam_pixel_fmt, // AV_PIX_FMT_BAYER_GBRG8, AV_PIX_FMT_RGB24
			out_xres, out_yres, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);

		if (!sws) {
			std::cout << "Cannot create SWS context" << std::endl;
		}
	}

	// Puts the raw char data of an image into an AV container, color converts and encodes it
	int encode(unsigned char* img) {

		// Put raw image data to AV picture
		int bytes_filled = av_image_fill_arrays(pic_raw.data, pic_raw.linesize, img,
												cam_pixel_fmt, in_xres, in_yres, 1);
		if (!bytes_filled) {
			std::cout << "Cannot fill the raw input buffer" << std::endl;
			return -1;
		}

		// convert to I420 for x264
		int h = sws_scale(sws, pic_raw.data, pic_raw.linesize, 0, in_yres, pic_in.img.plane,
						  pic_in.img.i_stride);
		if (h != out_yres) {
			std::cout << "scale failed: %d" << std::endl;
			;
			return -1;
		}

		// Encode
		pic_in.i_pts = framecounter++;
		int frame_size = x264_encoder_encode(enc, &nals, &num_nals, &pic_in, &pic_out);

		nals_simple.clear();
		for (auto i = 0; i < num_nals; i++) {
			x264_nal_t_simple nal;
			nal.i_payload = nals[i].i_payload;
			nal.p_payload = nals[i].p_payload;
			nals_simple.push_back(nal);
		}

		return frame_size;
	}
};

Encoder::Encoder(int inW, int inH, int outW, int outH, float fps, int rf) {
	impl = std::make_shared<EncoderImpl>(inW, inH, outW, outH, fps, rf);
}

int Encoder::encode(unsigned char* img) {
	return impl->encode(img);
}

const std::vector<x264_nal_t_simple>& Encoder::getNals() {
	return impl->nals_simple;
}

} // namespace h264encoder