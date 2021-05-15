#include "Camera.h"

using cv::Mat;
using cv::Size;
using std::string;

namespace cam
{
Camera::Camera()
	: _capture(std::make_shared<cv::VideoCapture>()), _frame(std::make_shared<cv::Mat>()),
	  _frame_num(std::make_shared<uint32_t>(0)), _frame_lock(std::make_shared<std::mutex>()),
	  _cap_lock(std::make_shared<std::mutex>())
{
}

bool Camera::open(int camera_id, CameraParams intrinsic_params, Mat extrinsic_params)
{
	if (_running)
	{
		return false;
	}
	_cap_lock->lock();
	bool result = this->_capture->open(camera_id);
	_cap_lock->unlock();
	this->_intrinsic_params = intrinsic_params;
	init(extrinsic_params);
	return result;
}

bool Camera::open(string filename, CameraParams intrinsic_params, Mat extrinsic_params)
{
	if (_running)
	{
		return false;
	}
	_cap_lock->lock();
	bool result = this->_capture->open(filename);
	_cap_lock->unlock();
	this->_intrinsic_params = intrinsic_params;
	init(extrinsic_params);
	return result;
}

Camera::Camera(string filename, string name, string description, CameraParams intrinsic_params,
			   Mat extrinsic_params)
	: _name(name), _description(description), _intrinsic_params(intrinsic_params),
	  _capture(std::make_shared<cv::VideoCapture>(filename)),
	  _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_lock(std::make_shared<std::mutex>()), _cap_lock(std::make_shared<std::mutex>())
{
	init(extrinsic_params);
}

Camera::Camera(int camera_id, string name, string description, CameraParams intrinsic_params,
			   Mat extrinsic_params)
	: _name(name), _description(description), _intrinsic_params(intrinsic_params),
	  _capture(std::make_shared<cv::VideoCapture>(camera_id)),
	  _frame(std::make_shared<cv::Mat>()), _frame_num(std::make_shared<uint32_t>(0)),
	  _frame_lock(std::make_shared<std::mutex>()), _cap_lock(std::make_shared<std::mutex>())
{
	init(extrinsic_params);
}

void Camera::init(const Mat &extrinsic_params)
{
	if (!extrinsic_params.empty() && extrinsic_params.size() != Size(4, 4))
	{
		throw std::invalid_argument("extrinsic_params must be 4x4 if given");
	}
	extrinsic_params.copyTo(this->_extrinsic_params);
	_running = std::make_shared<bool>(true);
	_thread = std::shared_ptr<std::thread>(new std::thread(&Camera::captureLoop, this),
										   [this](std::thread *p) {
											   *(this->_running) = false;
											   p->join();
											   delete p;
										   });
}

Camera::Camera(const Camera &other)
	: _running(other._running), _name(other._name), _description(other._description),
	  _frame(other._frame), _frame_num(other._frame_num), _capture(other._capture),
	  _extrinsic_params(other._extrinsic_params), _frame_lock(other._frame_lock),
	  _cap_lock(other._cap_lock), _thread(other._thread),
	  _intrinsic_params(other._intrinsic_params)
{
}

invalid_camera_config::invalid_camera_config() : _msg("Invalid camera configuration")
{
}

invalid_camera_config::invalid_camera_config(const string &msg)
	: _msg("Invalid camera configuration:" + msg)
{
}

const char *invalid_camera_config::what() const noexcept
{
	return _msg.c_str();
}

Camera Camera::openFromConfigFile(std::string filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	// read intrinsic parameters
	CameraParams intrinsics;
	if (!fs[KEY_INTRINSIC_PARAMS].empty())
	{
		fs[KEY_INTRINSIC_PARAMS] >> intrinsics;
	}

	// read extrinsic parameters
	cv::Mat extrinsics;
	if (!fs[KEY_EXTRINSIC_PARAMS].empty())
	{
		fs[KEY_EXTRINSIC_PARAMS] >> extrinsics;
	}

	// read name
	if (!fs[KEY_NAME].empty())
	{
		throw invalid_camera_config(KEY_NAME + " must be present");
	}
	string name = (string)fs[KEY_NAME];

	// read description
	string description;
	if (!fs[KEY_DESCRIPTION].empty())
	{
		description = (string)fs[KEY_DESCRIPTION];
	}

	// read filename or camera ID, and open camera.
	if (!fs[KEY_FILENAME].empty())
	{
		string cam_file = (string)fs[KEY_FILENAME];
		return Camera(cam_file, name, description, intrinsics, extrinsics);
	}
	else if (!fs[KEY_CAMERA_ID].empty())
	{
		int cam_id = (int)fs[KEY_CAMERA_ID];
		return Camera(cam_id, name, description, intrinsics, extrinsics);
	}
	else
	{
		throw invalid_camera_config("One of " + KEY_FILENAME + " or " + KEY_CAMERA_ID +
									" must be present");
	}
}

void Camera::captureLoop()
{
	cv::Size image_size(640, 480);
	if (!_intrinsic_params.empty())
	{
		image_size = _intrinsic_params.getImageSize();
	}
	_cap_lock->lock();
	_capture->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	_capture->set(cv::CAP_PROP_FRAME_WIDTH, image_size.width);
	_capture->set(cv::CAP_PROP_FRAME_HEIGHT, image_size.height);
	_cap_lock->unlock();
	cv::Mat frame;
	while (*_running)
	{
		_cap_lock->lock();
		_capture->read(frame);
		_cap_lock->unlock();
		_frame_lock->lock();
		frame.copyTo(*(this->_frame));
		(*_frame_num)++;
		_frame_lock->unlock();
	}
}

bool Camera::isOpen() const
{
	bool open;
	_cap_lock->lock();
	open = _capture->isOpened();
	_cap_lock->unlock();
	return open;
}

bool Camera::hasNext(uint32_t last_frame_num) const
{
	bool hasNext;
	_frame_lock->lock();
	hasNext = (last_frame_num != *_frame_num);
	_frame_lock->unlock();
	return hasNext;
}

bool Camera::next(cv::Mat &frame, uint32_t &frame_num) const
{
	if (!isOpen())
	{
		return false;
	}
	_frame_lock->lock();
	frame = this->_frame->clone();
	frame_num = *(this->_frame_num);
	_frame_lock->unlock();
	return true;
}

bool Camera::hasIntrinsicParams() const
{
	return !_intrinsic_params.empty();
}

bool Camera::hasExtrinsicParams() const
{
	return !_extrinsic_params.empty();
}

CameraParams Camera::getIntrinsicParams() const
{
	return _intrinsic_params;
}

cv::Mat Camera::getExtrinsicParams() const
{
	return _extrinsic_params.clone();
}

std::string Camera::getName() const
{
	return _name;
}

std::string Camera::getDescription() const
{
	return _description;
}

void Camera::setName(std::string new_name)
{
	this->_name = new_name;
}

void Camera::setDescription(std::string new_description)
{
	this->_description = new_description;
}

} // namespace cam
