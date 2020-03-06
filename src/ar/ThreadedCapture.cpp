#include "ThreadedCapture.h"

namespace AR
{

void ThreadedCapture::readLoop()
{
	while(running)
	{
		cv::Mat frame;
		cap.read(frame);
		mutex.lock();
		current_frame = frame;
		mutex.unlock();
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
	mutex.lock();
	frame = current_frame;
	mutex.unlock();
}

bool ThreadedCapture::set(cv::VideoCaptureProperties prop, double value)
{
	return cap.set(prop,value);
}

ThreadedCapture::~ThreadedCapture()
{
	running = false;
	thread.join();
	cap.release();
}

}
