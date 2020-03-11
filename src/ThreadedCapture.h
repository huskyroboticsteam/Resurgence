#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <thread>
#include <mutex>

namespace AR
{

class ThreadedCapture
{
private:
	std::thread thread;
	std::mutex frame_mutex;
	std::mutex cap_mutex;
	cv::Mat current_frame;
	cv::VideoCapture cap;
	bool running = false;
	void readLoop();
public:
	~ThreadedCapture();
	ThreadedCapture();
	bool set(cv::VideoCaptureProperties prop, double value);
	bool open(int cam_id, int api = cv::CAP_ANY);
	void read(cv::Mat& frame);
};

}
