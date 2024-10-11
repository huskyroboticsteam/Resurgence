#include "Camera.h"
#include "../Constants.h"

#include "CameraConfig.h"

#include <loguru.hpp>

#include <opencv2/aruco.hpp>

using cv::Mat;
using cv::Size;
using std::string;

using namespace robot::types;

namespace cam {
Camera::Camera()
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()),
	  _capture(),
	  _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _running(std::make_shared<bool>(false)) {}

bool Camera::open(int camera_id, CameraParams intrinsic_params, Mat extrinsic_params) {
	if (*_running) {
		return false;
	}
	_capture_lock->lock();
	std::stringstream gstr_ss = GStreamerFromFile(camera_id);
	this->_capture = std::make_shared<cv::VideoCapture>(gstr_ss.str(), cv::CAP_GSTREAMER);
	bool result = this->_capture->isOpened();
	_capture_lock->unlock();
	LOG_F(INFO, "Opening camera %d... %s", camera_id, result ? "success" : "failed");
	this->_intrinsic_params = intrinsic_params;
	init(extrinsic_params);
	return result;
}

Camera::Camera(string filename, string name, string description, CameraParams intrinsic_params,
			   Mat extrinsic_params)
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()),
	  _capture(std::make_shared<cv::VideoCapture>(filename)), _name(name),
	  _description(description), _frame_lock(std::make_shared<std::mutex>()),
	  _capture_lock(std::make_shared<std::mutex>()), _intrinsic_params(intrinsic_params),
	  _running(std::make_shared<bool>(false)) {
	init(extrinsic_params);
}

Camera::Camera(int camera_id, string name, string description, CameraParams intrinsic_params,
			   Mat extrinsic_params)
	: _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_time(std::make_shared<datatime_t>()),
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
		// return this->open(std::get<std::string>(cfg.filenameOrID), intrinsics, extrinsics);
	} else if (std::holds_alternative<int>(cfg.filenameOrID)) {
		return this->open(std::get<int>(cfg.filenameOrID), intrinsics, extrinsics);
	} else {
		// this should never happen
		throw invalid_camera_config("One of " + KEY_FILENAME + " or " + KEY_CAMERA_ID +
									" must be present");
	}
}

std::stringstream Camera::GStreamerFromFile(robot::types::CameraID camera_id) {
	cv::FileStorage fs(Constants::CAMERA_CONFIG_PATHS.at(camera_id), cv::FileStorage::READ);
	if (!fs.isOpened()) {
		throw std::invalid_argument("Configuration file of camera ID" + std::to_string(camera_id) + " does not exist");
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

	gstr_ss << "image/" << fs[KEY_FORMAT].operator std::string();
	gstr_ss << ",width=" << fs[KEY_WIDTH].operator std::string();
	gstr_ss << ",height=" << fs[KEY_HEIGHT].operator std::string();
	gstr_ss << ",framerate=" << fs[KEY_FRAMERATE].operator std::string() << "/1";
	gstr_ss << " ! " << fs[KEY_FORMAT].operator std::string() << "dec ! videoconvert";
	
	gstr_ss << " ! appsink";
	LOG_F(INFO, "GSTR: %s", gstr_ss.str().c_str());

	return gstr_ss;
}

void Camera::captureLoop() {
	loguru::set_thread_name(_name.c_str());
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
