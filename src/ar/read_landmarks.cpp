#include "read_landmarks.h"

#include "../Constants.h"
#include "../camera/Camera.h"
#include "../camera/CameraConfig.h"
#include "../world_interface/world_interface.h"
#include "Detector.h"

#include <atomic>
#include <loguru.hpp>
#include <map>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

using namespace robot::types;

namespace AR {
constexpr size_t NUM_LANDMARKS = 11;
constexpr std::chrono::milliseconds LANDMARK_FRESH_PERIOD(100);

const landmarks_t zero_landmarks(NUM_LANDMARKS);

Detector ar_detector(Markers::URC_MARKERS(), cam::CameraParams());

std::atomic<bool> fresh_data(false);
std::mutex landmark_lock;
landmarks_t current_landmarks(NUM_LANDMARKS);
std::thread landmark_thread;
bool initialized = false;

void detectLandmarksLoop() {
	loguru::set_thread_name("LandmarkDetection");
	cv::Mat frame;
	uint32_t last_frame_no = 0;
	while (true) {
		if (robot::hasNewCameraFrame(Constants::MAST_CAMERA_ID, last_frame_no)) {
			auto camData = robot::readCamera(Constants::MAST_CAMERA_ID);
			if (!camData)
				continue;
			auto camFrame = camData.getData();
			datatime_t frameTime = camData.getTime();
			frame = camFrame.first;
			last_frame_no = camFrame.second;

			std::vector<AR::Tag> tags = ar_detector.detectTags(frame);
			LOG_F(2, "readLandmarks(): %ld tags spotted", tags.size());

			// build up map with first tag of each ID spotted.
			landmarks_t output(NUM_LANDMARKS);
			std::map<int, cv::Vec3d> ids_to_camera_coords;
			for (AR::Tag tag : tags) {
				int id = tag.getMarker().getId();
				if (ids_to_camera_coords.find(id) == ids_to_camera_coords.end()) {
					ids_to_camera_coords[id] = tag.getCoordinates();
				}
			}

			// for every possible landmark ID, go through and if the map contains a value for
			// that ID, add it to the output array (doing appropriate coordinate space
			// transforms). If not, add a zero point.
			for (size_t i = 0; i < NUM_LANDMARKS; i++) {
				int id = static_cast<int>(i);
				if (ids_to_camera_coords.find(id) != ids_to_camera_coords.end()) {
					cv::Vec3d coords = ids_to_camera_coords[id];
					// if we have extrinsic parameters, multiply coordinates by them to do
					// appropriate transformation.
					auto extrinsic =
						robot::getCameraExtrinsicParams(Constants::MAST_CAMERA_ID);
					if (extrinsic) {
						cv::Vec4d coords_homogeneous = {coords[0], coords[1], coords[2], 1};
						cv::Mat transformed = extrinsic.value() * cv::Mat(coords_homogeneous);
						output[i] = {
							frameTime,
							{transformed.at<double>(0, 0), transformed.at<double>(1, 0), 1.0}};
					} else {
						// just account for coordinate axis change; rover frame has +x front,
						// +y left, +z up while camera has +x right, +y down, +z front.
						output[i] = {frameTime, {coords[2], -coords[0], 1.0}};
					}
				} else {
					output[i] = {};
				}
			}

			landmark_lock.lock();
			for (size_t i = 0; i < NUM_LANDMARKS; i++) {
				// only overwrite data if the new data is valid or the previous data is expired
				// detection is a bit spotty, so we don't want to overwrite good data witih
				// false negatives
				if (output[i].isValid() ||
					!current_landmarks[i].isFresh(LANDMARK_FRESH_PERIOD)) {
					current_landmarks[i] = output[i];
				}
			}
			fresh_data = true;
			landmark_lock.unlock();
		}
	}
}

bool initializeLandmarkDetection() {
	// Load camera intrinsic parameters directly from config file
	// This avoids opening the camera just to get its parameters
	try {
		auto config = cam::readConfigFromFile(Constants::CAMERA_CONFIG_PATHS.at(Constants::MAST_CAMERA_ID));
		if (!config.intrinsicParams || config.intrinsicParams->empty()) {
			LOG_F(ERROR, "Camera configuration does not have intrinsic parameters! "
						 "AR tag detection cannot be performed.");
			return false;
		}
		ar_detector = Detector(Markers::URC_MARKERS(), config.intrinsicParams.value());
		landmark_thread = std::thread(&detectLandmarksLoop);
		
		if (!config.extrinsicParams || config.extrinsicParams->empty()) {
			LOG_F(WARNING, "Camera configuration does not have extrinsic parameters! "
						   "Coordinates returned for AR tags will be relative to camera");
		}
	} catch (const std::exception& e) {
		LOG_F(ERROR, "Failed to load camera configuration: %s", e.what());
		return false;
	}
	
	initialized = true;
	return true;
}

bool isLandmarkDetectionInitialized() {
	return initialized;
}

landmarks_t readLandmarks() {
	if (isLandmarkDetectionInitialized()) {
		if (fresh_data) {
			landmarks_t output;
			landmark_lock.lock();
			output = current_landmarks;
			fresh_data = false;
			landmark_lock.unlock();
			return output;
		}
	}
	return zero_landmarks;
}

} // namespace AR
