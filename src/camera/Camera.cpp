#include "Camera.h"

using cv::Mat;
using cv::Size;
using std::string;

namespace cam
{
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

} // namespace cam