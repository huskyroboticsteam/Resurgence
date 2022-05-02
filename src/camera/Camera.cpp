#include "Camera.h"

#include "CameraConfig.h"

#include <iostream>

using cv::Mat;
using cv::Size;
using std::string;

using namespace robot::types;

namespace cam {
Camera::Camera()
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _capture(std::make_shared<cv::VideoCapture>()),
	  _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _running(std::make_shared<bool>(false)) {}

bool Camera::open(int camera_id, CameraParams intrinsic_params, Mat extrinsic_params) {
	if (*_running) {
		return false;
	}
	_capture_lock->lock();
	std::cout << "Opening camera " << camera_id << "... ";
	bool result = this->_capture->open(camera_id);
	std::cout << (result ? "success" : "failed") << std::endl;
	_capture_lock->unlock();
	this->_intrinsic_params = intrinsic_params;
	init(extrinsic_params);
	return result;
}

bool Camera::open(string filename, CameraParams intrinsic_params, Mat extrinsic_params) {
	if (*_running) {
		return false;
	}
	_capture_lock->lock();
	bool result = this->_capture->open(filename);
	_capture_lock->unlock();
	this->_intrinsic_params = intrinsic_params;
	init(extrinsic_params);
	return result;
}

Camera::Camera(string filename, string name, string description, CameraParams intrinsic_params,
			   Mat extrinsic_params)
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _capture(std::make_shared<cv::VideoCapture>(filename)), _name(name),
	  _description(description), _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _intrinsic_params(intrinsic_params),
	  _running(std::make_shared<bool>(false)) {
	init(extrinsic_params);
}

Camera::Camera(int camera_id, string name, string description, CameraParams intrinsic_params,
			   Mat extrinsic_params)
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _capture(std::make_shared<cv::VideoCapture>(camera_id)), _name(name),
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
				std::cout << "shutting down camera thread" << std::endl;
				*(this->_running) = false;
				p->join();
			}
			delete p;
		});
}

Camera::Camera(const Camera& other)
	: _frame(other._frame), _frame_num(other._frame_num), _capture(other._capture),
	  _name(other._name), _description(other._description), _frame_lock(other._frame_lock),
	  _capture_lock(other._capture_lock), _thread(other._thread),
	  _intrinsic_params(other._intrinsic_params), _extrinsic_params(other._extrinsic_params),
	  _running(other._running) {}

bool Camera::openFromConfigFile(std::string filename) {
	CameraConfig cfg = readConfigFromFile(filename);

	cv::Mat extrinsics;
	if (cfg.extrinsicParams) {
		extrinsics = cfg.extrinsicParams.value();
	}

	CameraParams intrinsics;
	if (cfg.intrinsicParams) {
		intrinsics = cfg.intrinsicParams.value();
	}

	if (std::holds_alternative<std::string>(cfg.filenameOrID)) {
		return this->open(std::get<std::string>(cfg.filenameOrID), intrinsics, extrinsics);
	} else if (std::holds_alternative<int>(cfg.filenameOrID)) {
		return this->open(std::get<int>(cfg.filenameOrID), intrinsics, extrinsics);
	} else {
		// this should never happen
		throw invalid_camera_config("One of " + KEY_FILENAME + " or " + KEY_CAMERA_ID +
									" must be present");
	}
}

void Camera::captureLoop() {
	cv::Size image_size(640, 480);
	if (!_intrinsic_params.empty()) {
		image_size = _intrinsic_params.getImageSize();
	}
	_capture_lock->lock();
	_capture->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	_capture->set(cv::CAP_PROP_FRAME_WIDTH, image_size.width);
	_capture->set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
	_capture_lock->unlock();
	cv::Mat frame;
	while (*_running) {
		_capture_lock->lock();
		_capture->read(frame);
		_capture_lock->unlock();
		_frame_lock->lock();
		frame.copyTo(*(this->_frame));
		(*_frame_num)++;
		*_frame_time = dataclock::now();
		_frame_lock->unlock();
	}
}

bool Camera::isOpen() const {
	bool open;
	_capture_lock->lock();
	open = _capture->isOpened();
	_capture_lock->unlock();
	return open;
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
