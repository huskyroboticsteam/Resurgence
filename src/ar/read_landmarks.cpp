#include "read_landmarks.h"

#include "../Constants.h"
#include "../camera/Camera.h"
#include "../log.h"
#include "../world_interface/world_interface.h"
#include "Detector.h"

#include <atomic>
#include <map>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>

namespace AR {
constexpr size_t NUM_LANDMARKS = 11;
const point_t ZERO_POINT = {0.0, 0.0, 0.0};
static points_t make_zero_landmarks() {
	points_t z;
	for (size_t i = 0; i < NUM_LANDMARKS; i++) {
		z.push_back(ZERO_POINT);
	}
	return z;
}
const points_t zero_landmarks = make_zero_landmarks();

Detector ar_detector(Markers::URC_MARKERS(), cam::CameraParams());

std::atomic<bool> fresh_data(false);
std::mutex landmark_lock;
points_t current_landmarks;
std::thread landmark_thread;
bool initialized = false;

void detectLandmarksLoop() {
	cv::Mat frame;
	uint32_t last_frame_no = 0;
	while (true) {
		if (hasNewCameraFrame(Constants::AR_CAMERA_ID, last_frame_no)) {
			auto camData = readCamera(Constants::AR_CAMERA_ID);
			if (!camData) continue;
			auto camFrame = camData.getData();
			frame = camFrame.first;
			last_frame_no = camFrame.second;

			std::vector<AR::Tag> tags = ar_detector.detectTags(frame);
			log(LOG_DEBUG, "readLandmarks(): %d tags spotted\n", tags.size());

			// build up map with first tag of each ID spotted.
			points_t output(NUM_LANDMARKS);
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
					auto extrinsic = getCameraExtrinsicParams(Constants::AR_CAMERA_ID);
					if (extrinsic) {
						cv::Vec4d coords_homogeneous = {coords[0], coords[1], coords[2], 1};
						cv::Mat transformed = extrinsic.value() * cv::Mat(coords_homogeneous);
						output[i] = {transformed.at<double>(0, 0),
									 transformed.at<double>(1, 0), 1.0};
					} else {
						// just account for coordinate axis change; rover frame has +x front,
						// +y left, +z up while camera has +x right, +y down, +z front.
						output[i] = {coords[2], -coords[0], 1.0};
					}
				} else {
					output[i] = ZERO_POINT;
				}
			}

			landmark_lock.lock();
			for (size_t i = 0; i < NUM_LANDMARKS; i++) {
				// If we already saw this landmark, we don't want to overwrite that with zeros
				// if we didn't see the landmark on this particular frame. This is because
				// landmark detection is a bit spotty: even when the rover is not moving,
				// sometimes an AR tag is only detected in 40% or 50% of frames.
				//
				// TODO: We want to make sure that this data eventually *does* expire, so that
				// if the rover moves or turns significantly, it won't interpret the landmark
				// location as being relative to the new rover location.
				//
				// Note that when we call readLandmarks(), we do zero out the data. But we
				// don't want that to be the exclusive way to prevent data from getting stale.
				if (output[i](2) != 0.0)
					current_landmarks[i] = output[i];
			}
			fresh_data = true;
			landmark_lock.unlock();
		}
	}
}

void zeroCurrent() {
	current_landmarks.clear();
	for (size_t i = 0; i < NUM_LANDMARKS; i++) {
		current_landmarks.push_back(ZERO_POINT);
	}
}

bool initializeLandmarkDetection() {
	zeroCurrent();
	auto intrinsic = getCameraIntrinsicParams(Constants::AR_CAMERA_ID);
	if (intrinsic) {
		ar_detector = Detector(Markers::URC_MARKERS(), intrinsic.value());
		landmark_thread = std::thread(&detectLandmarksLoop);
	} else {
		log(LOG_ERROR, "Camera does not have intrinsic parameters! AR tag detection "
						"cannot be performed.\n");
		return false;
	}
	if (!getCameraExtrinsicParams(Constants::AR_CAMERA_ID)) {
		log(LOG_WARN, "Camera does not have extrinsic parameters! Coordinates returned "
						"for AR tags will be relative to camera\n");
	}
	initialized = true;
	return true;
}

bool isLandmarkDetectionInitialized() {
	return initialized;
}

points_t readLandmarks() {
	if (isLandmarkDetectionInitialized()) {
		if (fresh_data) {
			points_t output;
			landmark_lock.lock();
			output = current_landmarks;
			fresh_data = false;
			zeroCurrent();
			landmark_lock.unlock();
			return output;
		}
	}
	return zero_landmarks;
}

} // namespace AR
