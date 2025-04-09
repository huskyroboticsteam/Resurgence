#include "Camera.h"

#include "../Constants.h"

#include <loguru.hpp>

#include <opencv2/aruco.hpp>

using cv::Mat;
using cv::Size;
using std::string;

using namespace robot::types;

namespace cam {
Camera::Camera()
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()), _capture(),
	  _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _running(std::make_shared<bool>(false)) {}

Camera::Camera(robot::types::CameraID camera_id)
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()),
	  _capture(std::make_shared<cv::VideoCapture>(camera_id)),
	  _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _running(std::make_shared<bool>(false)) {
	this->open(camera_id);
}

Camera::Camera(const Camera& other)
	: _frame(other._frame), _frame_num(other._frame_num), _frame_time(other._frame_time),
	  _capture(other._capture), _name(other._name), _description(other._description),
	  _frame_lock(other._frame_lock), _capture_lock(other._capture_lock),
	  _thread(other._thread), _running(other._running) {}

bool Camera::open(robot::types::CameraID camera_id) {
	if (*_running) {
		return false;
	}
	_capture_lock->lock();
	std::stringstream gstr_ss = GStreamerFromFile(camera_id);
	this->_capture = std::make_shared<cv::VideoCapture>(gstr_ss.str(), cv::CAP_GSTREAMER);
	bool result = this->_capture->isOpened();
	_capture_lock->unlock();
	LOG_F(INFO, "Opening camera %d... %s", camera_id, result ? "success" : "failed");
	init();
	return result;
}

void Camera::init() {
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

std::stringstream Camera::GStreamerFromFile(robot::types::CameraID camera_id) {
	cv::FileStorage fs(Constants::CAMERA_CONFIG_PATHS.at(camera_id).data(),
					   cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::invalid_argument("Configuration file of camera ID" +
									std::to_string(camera_id) + " does not exist");
	}

	std::stringstream gstr_ss;
	gstr_ss << "v4l2src device=/dev/video" << camera_id << " ! ";

	const std::string KEY_NAME = "name";
	const std::string KEY_DESCRIPTION = "description";
	const std::string KEY_CAMERA_ID = "camera_id";
	const std::string KEY_FORMAT = "format";
	const std::string KEY_WIDTH = "width";
	const std::string KEY_HEIGHT = "height";
	const std::string KEY_FRAMERATE = "framerate";

	_name = fs[KEY_NAME].operator std::string();
	_description = fs[KEY_DESCRIPTION].operator std::string();
	_width = fs[KEY_WIDTH].operator int();
	_height = fs[KEY_HEIGHT].operator int();

	gstr_ss << "image/" << fs[KEY_FORMAT].operator std::string();
	gstr_ss << ",width=" << _width;
	gstr_ss << ",height=" << _height;
	gstr_ss << ",framerate=" << fs[KEY_FRAMERATE].operator std::string() << "/1";
	gstr_ss << " ! " << fs[KEY_FORMAT].operator std::string() << "dec ! videoconvert";

	gstr_ss << " ! appsink";
	LOG_F(INFO, "GSTR: %s", gstr_ss.str().c_str());

	return gstr_ss;
}

void Camera::captureLoop() {
	loguru::set_thread_name(_name.c_str());
	cv::Mat frame;
	while (*_running) {
		_capture_lock->lock();
		bool success = _capture->read(frame);
		_capture_lock->unlock();
		if (success && !frame.empty()) {
			// Aruco detection here:
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

std::string Camera::getName() const {
	return _name;
}

std::string Camera::getDescription() const {
	return _description;
}

int Camera::getWidth() const {
	return _width;
}

int Camera::getHeight() const {
	return _height;
}

void Camera::setName(std::string new_name) {
	this->_name = new_name;
}

void Camera::setDescription(std::string new_description) {
	this->_description = new_description;
}

} // namespace cam
