#include "read_landmarks.h"
#include "../camera/Camera.h"
#include "Detector.h"
#include "../Constants.h"
#include "../log.h"
#include <mutex>
#include <thread>
#include <atomic>
#include <opencv2/core.hpp>
#include <map>

namespace AR
{
constexpr size_t NUM_LANDMARKS = 11;
const point_t ZERO_POINT = {0.0, 0.0, 0.0};
constexpr auto ZERO_DURATION = std::chrono::microseconds(0);
static points_t make_zero_landmarks(){
	points_t z;
	for(int i = 0; i < NUM_LANDMARKS; i++){
		z.push_back(ZERO_POINT);
	}
	return z;
}
const points_t zero_landmarks = make_zero_landmarks();

cam::Camera ar_cam;
Detector ar_detector(Markers::URC_MARKERS(), cam::CameraParams());

std::atomic<bool> fresh_data(false);
std::mutex landmark_lock;
points_t current_landmarks;
std::thread landmark_thread;

void detectLandmarksLoop(){
	cv::Mat frame;
	uint32_t last_frame_no;
	while(true){
		if (ar_cam.hasNext(last_frame_no)) {
			ar_cam.next(frame, last_frame_no);
	
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
			for (int i = 0; i < NUM_LANDMARKS; i++) {
				if (ids_to_camera_coords.find(i) != ids_to_camera_coords.end()) {
					cv::Vec3d coords = ids_to_camera_coords[i];
					// if we have extrinsic parameters, multiply coordinates by them to do
					// appropriate transformation.
					if (ar_cam.hasExtrinsicParams()) {
						cv::Vec4d coords_homogeneous = {coords[0], coords[1], coords[2], 1};
						cv::Mat transformed =
							ar_cam.getExtrinsicParams() * cv::Mat(coords_homogeneous);
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
			for (int i = 0; i < NUM_LANDMARKS; i++) {
				if (output[i](2) != 0.0) current_landmarks[i] = output[i];
			}
			fresh_data = true;
			landmark_lock.unlock();
		}
	}
}

void zeroCurrent() {
	current_landmarks.clear();
	for (int i = 0; i < NUM_LANDMARKS; i++) {
		current_landmarks.push_back(ZERO_POINT);
	}
}

bool initializeLandmarkDetection(){
	zeroCurrent();
	try {
		ar_cam.openFromConfigFile(Constants::AR_CAMERA_CONFIG_PATH);
		if(!ar_cam.hasIntrinsicParams()){
			log(LOG_ERROR, "Camera does not have intrinsic parameters! AR tag detection "
						   "cannot be performed.\n");
			return false;
		} else {
			ar_detector =
				Detector(Markers::URC_MARKERS(), ar_cam.getIntrinsicParams());
			landmark_thread = std::thread(&detectLandmarksLoop);
		}
		if(!ar_cam.hasExtrinsicParams()){
			log(LOG_WARN, "Camera does not have extrinsic parameters! Coordinates returned "
						  "for AR tags will be relative to camera\n");
		}
		return true;
	} catch (std::exception& e) {
		log(LOG_ERROR, "Error opening camera for AR tag detection:\n%s\n", e.what());
		return false;
	}
}

bool isLandmarkDetectionInitialized(){
	return ar_cam.isOpen();
}

points_t readLandmarks(){
	if(isLandmarkDetectionInitialized()){
		if (fresh_data)	{
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

}
