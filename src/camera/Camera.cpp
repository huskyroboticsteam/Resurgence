#include "Camera.h"

#include "../Constants.h"
#include "CameraConfig.h"

#include <loguru.hpp>

#include <opencv2/aruco.hpp>

#include <unistd.h> 
#include <loguru.hpp> 


using cv::Mat;
using cv::Size;
using std::string;

using namespace robot::types;

namespace cam {
Camera::Camera()
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()), _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _running(std::make_shared<bool>(false)) {}

bool Camera::open(CameraID camera_id, CameraParams intrinsic_params, Mat extrinsic_params) {
	if (*_running) {
		return false;
	}
	_capture_lock->lock();
	std::string gstr = getGSTPipe(camera_id);
	LOG_F(INFO, "GST: %s", gstr.c_str());
	this->_capture = std::make_shared<cv::VideoCapture>(gstr, cv::CAP_GSTREAMER);
	bool result = this->_capture->isOpened();
	_capture_lock->unlock();
	LOG_F(INFO, "Opening %s camera... %s", camera_id.c_str(), result ? "success" : "failed");
	this->_intrinsic_params = intrinsic_params;
	init(extrinsic_params);
	return result;
}

Camera::Camera(CameraID camera_id, string name, string description,
			   CameraParams intrinsic_params, Mat extrinsic_params)
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()),
	  _capture(std::make_shared<cv::VideoCapture>(getGSTPipe(camera_id), cv::CAP_GSTREAMER)),
	  _name(name), _description(description), _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _intrinsic_params(intrinsic_params),
	  _running(std::make_shared<bool>(false)) {
	init(extrinsic_params);
}

void Camera::init(const Mat& extrinsic_params) {
	if (!extrinsic_params.empty() && extrinsic_params.size() != Size(4, 4)) {
		throw std::invalid_argument("extrinsic_params must be 4x4 if given");
	}
	extrinsic_params.copyTo(this->_extrinsic_params);
	*(this->_running) = true;
	this->_thread = std::shared_ptr<std::thread>(
		new std::thread(&Camera::captureLoop, this), [this](std::thread* p) {
			if (*(this->_running)) {
				LOG_F(INFO, "Shutting down camera thread");
				*(this->_running) = false;
				p->join();
			}
			delete p;
		});
}

Camera::Camera(const Camera& other)
	: _frame(other._frame), _frame_num(other._frame_num), _frame_time(other._frame_time),
	  _capture(other._capture), _name(other._name), _description(other._description),
	  _frame_lock(other._frame_lock), _capture_lock(other._capture_lock),
	  _thread(other._thread), _intrinsic_params(other._intrinsic_params),
	  _extrinsic_params(other._extrinsic_params), _running(other._running) {}

#include <unistd.h>     // for access(), F_OK
#include <loguru.hpp>   // for LOG_F
#include <atomic>
#include <sstream>
#include <stdexcept>

std::string Camera::getGSTPipe(CameraID camera_id) {
    // Shared flag across all threads — ensures only one camera opens
    static std::atomic<bool> camera_in_use{false};

    int cam_id = 2;  // hardcoded to your working /dev/video2
    std::string device_path = "/dev/video" + std::to_string(cam_id);

    // Skip if camera already opened successfully
    if (camera_in_use.load()) {
        LOG_F(WARNING, "Camera already in use — skipping for CameraID %d", static_cast<int>(camera_id));
        return "";
    }

    // Check if the camera device exists
    if (access(device_path.c_str(), F_OK) == -1) {
        LOG_F(WARNING, "Camera device not found: %s — skipping pipeline creation.", device_path.c_str());
        return "";
    }

    // Build GPU-accelerated H.264 GStreamer pipeline
    std::stringstream gstr_ss;
    gstr_ss << "v4l2src device=" << device_path << " io-mode=2 ! ";
    gstr_ss << "video/x-h264,width=1920,height=1080,framerate=30/1 ! ";
    gstr_ss << "h264parse ! nvv4l2decoder ! nvvidconv ! ";
    gstr_ss << "video/x-raw,format=BGRx ! videoconvert ! appsink";

    LOG_F(INFO, "Opening camera on %s with GPU pipeline", device_path.c_str());

    // Mark the camera as in use (prevents other threads from trying again)
    camera_in_use.store(true);

    return gstr_ss.str();
}


// std::string Camera::getGSTPipe(CameraID camera_id) {
//     cv::FileStorage fs(Constants::CAMERA_CONFIG_PATHS.at(camera_id), cv::FileStorage::READ);
//     if (!fs.isOpened()) {
//         throw std::invalid_argument("Configuration file for Camera ID " + camera_id + " does not exist");
//     }

//     if (fs[KEY_IMAGE_WIDTH].empty() || fs[KEY_IMAGE_HEIGHT].empty() || fs[KEY_FRAMERATE].empty()) {
//         throw std::invalid_argument("Configuration file missing key(s)");
//     }

//     std::stringstream gstr_ss;
//     std::string format = fs[KEY_FORMAT];
//     int cam_id = static_cast<int>(fs[KEY_CAMERA_ID]);
//     std::string device_path = "/dev/video" + std::to_string(cam_id);

//     // ✅ Check camera device existence first
//     if (access(device_path.c_str(), F_OK) == -1) {
//         LOG_F(WARNING, "Camera device not found: %s — skipping pipeline creation.", device_path.c_str());
//         return "";
//     }

//     gstr_ss << "v4l2src device=" << device_path << " io-mode=2 ! ";
//     gstr_ss << format << ",";
//     gstr_ss << "width=" << static_cast<int>(fs[KEY_IMAGE_WIDTH]);
//     gstr_ss << ",height=" << static_cast<int>(fs[KEY_IMAGE_HEIGHT]);
//     gstr_ss << ",framerate=" << static_cast<int>(fs[KEY_FRAMERATE]) << "/1 ! ";

//     if (format == "image/jpeg") {
//         // GPU hardware MJPEG decode path
//         gstr_ss << "jpegparse ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! ";
//     } else {
//         // Non-MJPEG fallback
//         gstr_ss << "videoconvert ! ";
//     }

//     gstr_ss << "appsink";
//     return gstr_ss.str();
// }





void Camera::captureLoop() {
	loguru::set_thread_name(_name.c_str());
	cv::Mat frame;
	while (*_running) {
		_capture_lock->lock();
		bool success = _capture->read(frame);
		_capture_lock->unlock();
		if (success && !frame.empty()) {
			cv::Ptr<cv::aruco::Dictionary> dictionary =
				cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
			std::vector<std::vector<cv::Point2f>> markerCorners;
			cv::Mat frameCopy;
			std::vector<int> markerIds;
			cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
			frame.copyTo(frameCopy);
			if (!markerIds.empty()) {
				cv::aruco::drawDetectedMarkers(frameCopy, markerCorners, markerIds);
			}

			_frame_lock->lock();
			frameCopy.copyTo(*(this->_frame));
			(*_frame_num)++;
			*_frame_time = dataclock::now();
			_frame_lock->unlock();
		}
	}
}

bool Camera::isOpen() const {
	return *_running;
}

bool Camera::hasNext(uint32_t last_frame_num) const {
	bool hasNext;
	_frame_lock->lock();
	hasNext = (last_frame_num != *_frame_num);
	_frame_lock->unlock();
	return hasNext;
}

bool Camera::next(cv::Mat& frame, uint32_t& frame_num) const {
	if (!isOpen()) {
		return false;
	}
	_frame_lock->lock();
	frame = this->_frame->clone();
	frame_num = *(this->_frame_num);
	_frame_lock->unlock();
	return true;
}

bool Camera::next(cv::Mat& frame, uint32_t& frame_num, datatime_t& frame_time) const {
	if (!isOpen()) {
		return false;
	}
	_frame_lock->lock();
	frame = this->_frame->clone();
	frame_num = *(this->_frame_num);
	frame_time = *_frame_time;
	_frame_lock->unlock();
	return true;
}

bool Camera::hasIntrinsicParams() const {
	return !(_intrinsic_params.empty());
}

bool Camera::hasExtrinsicParams() const {
	return !(_extrinsic_params.empty());
}

CameraParams Camera::getIntrinsicParams() const {
	return _intrinsic_params;
}

cv::Mat Camera::getExtrinsicParams() const {
	return _extrinsic_params.clone();
}

std::string Camera::getName() const {
	return _name;
}

std::string Camera::getDescription() const {
	return _description;
}

void Camera::setName(std::string new_name) {
	this->_name = new_name;
}

void Camera::setDescription(std::string new_description) {
	this->_description = new_description;
}

} // namespace cam
