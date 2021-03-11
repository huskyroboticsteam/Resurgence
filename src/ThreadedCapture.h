#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <thread>
#include <mutex>

namespace AR
{

/**
   Class for multithreaded video capture using OpenCV. Can be used in a manner similar to
   an OpenCV VideoCapture class, but uses a separate thread to continually refresh the
   camera so that a frame is always available for processing.
 */
class ThreadedCapture
{
private:
	/**
	   The thread refreshing the camera
	 */
	std::thread thread;
	/**
	   Mutex to prevent concurrent modification of the current frame
	 */
	std::mutex frame_mutex;
	/**
	   Mutex to prevent concurrent modification of the VideoCapture object
	 */
	std::mutex cap_mutex;
	/**
	   The current frame
	 */
	cv::Mat current_frame;
	/**
	   The VideoCapture object used to access the camera
	 */
	cv::VideoCapture cap;
	/**
	   Whether or not the internal loop should continue to run
	 */
	bool running = false;
	/**
	   The internal loop that refreshes the camera, runs in a separate thread
	 */
	void readLoop();
	/**
	   The number of the current frame.
	 */
	size_t frame_num;
public:
	~ThreadedCapture();
	ThreadedCapture();
	/**
	   Sets a property on the underlying VideoCapture.
	   Can be used equivalently to VideoCapture::set, but please note that properties
	   should be set AFTER calling ThreadedCapture::open.
	 */
	bool set(cv::VideoCaptureProperties prop, double value);
	/**
	   Gets a property from the underlying VideoCapture. Can be used equivalently to
	   VideoCapture::get.
	 */
	double get(cv::VideoCaptureProperties prop);
	/**
	   Opens the camera and begins the capture thread.
	   @param cam_id The camera ID, if set to 0, the system will use the default camera.
	   @param api The API to use for video capture, by default the system will choose the
	   most appropriate API.
	 */
	bool open(int cam_id, int api = cv::CAP_ANY);
	/**
	   Reads the current frame.
	   @param frame A reference to an OpenCV Mat object, into which the current frame will
	   be stored.
	   @return Returns the number of the frame read.
	 */
	size_t read(cv::Mat& frame);
	/**
	   Returns whether or not the capture object has a new frame, relative to the frame number
	   given.
	   @param last_num The number of the last frame read.
	   @return true if a new frame (relative to last_num) has been read, and false if not
	   (i.e. if the current frame number is the same as last_num).
	 */
	bool hasNewFrame(size_t last_num);
};

}
