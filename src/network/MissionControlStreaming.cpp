#include "../base64/base64_img.h"
#include "../log.h"
#include "../world_interface/world_interface.h"
#include "MissionControlProtocol.h"

namespace net {
namespace mc {
void MissionControlProtocol::videoStreamTask() {
	while (this->_streaming_running) {
		std::shared_lock<std::shared_mutex> stream_lock(this->_stream_mutex);
		// for all open streams, check if there is a new frame
		for (const auto& stream : _open_streams) {
			const CameraID& cam = stream.first;
			const uint32_t& frame_num = stream.second;
			if (robot::hasNewCameraFrame(cam, frame_num)) {
				// if there is a new frame, grab it
				auto data = robot::readCamera(cam).getData();
				uint32_t& new_frame_num = data.second;
				cv::Mat frame = data.first;
				// update the previous frame number
				this->_open_streams[cam] = new_frame_num;

				// convert frame to base64 and send it
				std::string b64_data = base64::encodeMat(frame, ".jpg");
				sendCameraStreamReport(cam, b64_data);
			}

			// break out of the loop if we should stop streaming
			if (!this->_streaming_running) {
				break;
			}
		}
	}
}
} // namespace mc
} // namespace net
