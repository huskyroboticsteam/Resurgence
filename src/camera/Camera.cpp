#include "../Constants.h"
#include "Camera.h"
#include "CameraConfig.h"

#include <loguru.hpp>

using cv::Mat;
using cv::Size;
using std::string;

using namespace robot::types;

namespace cam {
Camera::Camera()
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()),
	  _frame_lock(std::make_shared<std::mutex>()),
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
	LOG_F(INFO, "Opening camera %d... %s", camera_id, result ? "success" : "failed");
	this->_intrinsic_params = intrinsic_params;
	init(extrinsic_params);
	return result; 
}

Camera::Camera(CameraID camera_id, string name, string description, CameraParams intrinsic_params,
			   Mat extrinsic_params)
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()),
	  _capture(std::make_shared<cv::VideoCapture>(getGSTPipe(camera_id), cv::CAP_GSTREAMER)), _name(name),
	  _description(description), _frame_lock(std::make_shared<std::mutex>()),
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

std::string Camera::getGSTPipe(CameraID camera_id) {
	cv::FileStorage fs(Constants::CAMERA_CONFIG_PATHS.at(camera_id), cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::invalid_argument("Configuration file for Camera ID" + std::to_string(camera_id) + " does not exist");
	}

	const std::string KEY_CAMERA_IMAGE_WIDTH = "image_width";
	const std::string KEY_CAMERA_IMAGE_HEIGHT = "image_height";
	const std::string KEY_CAMERA_FRAMERATE = "framerate";

	if (fs[KEY_CAMERA_IMAGE_WIDTH].empty() || fs[KEY_CAMERA_IMAGE_HEIGHT].empty() || fs[KEY_CAMERA_FRAMERATE].empty()) {
		throw std::invalid_argument("Configuration file missing key(s)");
	}

	std::stringstream gstr_ss;
	gstr_ss << "v4l2src device=/dev/video" << camera_id << " ! ";
	gstr_ss << "image/jpeg,width=" << static_cast<int>(fs[KEY_CAMERA_IMAGE_WIDTH]);
	gstr_ss << ",height=" << static_cast<int>(fs[KEY_CAMERA_IMAGE_HEIGHT]);
	gstr_ss << ",framerate=" << static_cast<int>(fs[KEY_CAMERA_FRAMERATE]) << "/1 ! ";
	gstr_ss << "jpegdec ! videoconvert ! appsink";

	return gstr_ss.str();
}

void Camera::captureLoop() {
	loguru::set_thread_name(_name.c_str());
	cv::Mat frame;
	while (*_running) {
		_capture_lock->lock();
		bool success = _capture->read(frame);
		_capture_lock->unlock();
		if (success && !frame.empty()) {
			_frame_lock->lock();
			frame.copyTo(*(this->_frame));
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
