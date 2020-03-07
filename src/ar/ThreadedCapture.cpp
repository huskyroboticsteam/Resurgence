#include "ThreadedCapture.h"

#include <iostream>

namespace AR
{

void ThreadedCapture::readLoop()
{
	while(running)
	{
		cv::Mat frame;
		
		cap_mutex.lock();
		cap.read(frame);
		cap_mutex.unlock();
		
		frame_mutex.lock();
		current_frame = frame;
		frame_mutex.unlock();
	}
}

bool ThreadedCapture::open(int cam_id, int api)
{
	cap.open(cam_id+api);
	if(!cap.isOpened())
	{
		return false;
	}
	thread = std::thread(&ThreadedCapture::readLoop, this);
	running = true;
	return true;
}

void ThreadedCapture::read(cv::Mat& frame)
{
	frame_mutex.lock();
	frame = current_frame;
	frame_mutex.unlock();
}

bool ThreadedCapture::set(cv::VideoCaptureProperties prop, double value)
{
	cap_mutex.lock();
	bool val = cap.set(prop,value);
	//   std::cout << val << ") set " << prop << ": " << value << ", actual val: "
	//   		  << cap.get(prop) << std::endl;
	cap_mutex.unlock();

	return val;
}

ThreadedCapture::~ThreadedCapture()
{
	running = false;
	thread.join();
	cap.release();
}

ThreadedCapture::ThreadedCapture()
{
	cv::VideoCapture _cap;
	this->cap = _cap;
}

}
