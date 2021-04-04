#include "ThreadedCapture.h"

#include <iostream>

namespace AR
{

void ThreadedCapture::readLoop()
{
	cv::Mat frame;
	while(running)
	{
		cap_mutex.lock();
		cap.read(frame);
		cap_mutex.unlock();
		
		frame_mutex.lock();
		frame.copyTo(current_frame);
		frame_num++;
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
	frame_num = 0;
	thread = std::thread(&ThreadedCapture::readLoop, this);
	running = true;
	return true;
}

size_t ThreadedCapture::read(cv::Mat& frame)
{
	size_t fnum;
	frame_mutex.lock();
	current_frame.copyTo(frame);
	fnum = frame_num;
	frame_mutex.unlock();
	return fnum;
}

bool ThreadedCapture::hasNewFrame(size_t last_num) {
	bool res;
	frame_mutex.lock();
	res = (last_num < frame_num);
	frame_mutex.unlock();
	return res;
}

bool ThreadedCapture::set(cv::VideoCaptureProperties prop, double value)
{
	cap_mutex.lock();
	bool val = cap.set(prop,value);
	cap_mutex.unlock();

	return val;
}

double ThreadedCapture::get(cv::VideoCaptureProperties prop)
{
	cap_mutex.lock();
	double val = cap.get(prop);
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
{}

}
