#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "CameraParams.h"

/**
   @defgroup camera Camera Access
   @brief This module provides an interface for defining and accessing the
   different cameras attached to the rover.

   Cameras should be defined in a YAML file with a format that will be
   documented soon, and the location of this file should be able to be
   optionally specified at runtime.
 */

namespace cam
{

/**
   @addtogroup camera
   @{
 */

/**
	@brief Represents a camera on the rover, from which a video feed can be
	retrieved.

	The Camera class provides an interface for opening a camera attached to the
	rover, and retrieving frames from it. Additionally, a Camera object contains
	other information associated with the camera, such as a description, its
	location on the rover, and its intrinsic camera parameters.

	A Camera object provides concurrent access to the camera; that is, the
	video feed will be updated in a separate thread and a client may retrieve a
	frame at any time. To avoid getting duplicate frames, there is a concept of
	a "frame number"; it will be returned upon retrieving a frame, and may be
	used to check if the current frame held by the Camera is different from the
	last one the client retrieved. The frame number will be different if two
	frames (within a reasonable time interval) are different.

	Additionally, if copied, Camera objects share ownership over a camera; when
	the last Camera object is destroyed, the underlying camera will be
	closed. This means multiple Camera objects may exist at different places in
	the code that refer to the same underlying camera and share access to it.
 */
class Camera
{
private:
	std::shared_ptr<cv::Mat> _frame;
	std::shared_ptr<uint32_t> _frame_num;
	std::shared_ptr<cv::VideoCapture> _capture;
	std::string _name;
	std::string _description;
	std::shared_ptr<std::mutex> _frame_lock;
	std::shared_ptr<std::mutex> _cap_lock;
	std::shared_ptr<std::thread> _thread;
	CameraParams _intrinsic_params;
	cv::Mat _extrinsic_params;
	std::shared_ptr<bool> _running;
	void captureLoop();
	void init(const cv::Mat &extrinsic_params);

public:
	/**
	   Constructs a Camera that will open the given file for input and have the
	   given name and description.

	   @param filename The file to open and read a video feed from. This may be
	   the name of a video file, or a URI of a video stream. This will be passed
	   to the underlying OpenCV VideoCapture object, so anything supported by
	   VideoCapture is supported here.

	   @param name The name of the camera. This should ideally be unique, but
	   isn't enforced at this time.

	   @param description An optional description of the camera.
	 */
	Camera(std::string filename, std::string name, std::string description = "",
		   CameraParams intrinsic_params = CameraParams(),
		   cv::Mat extrinsic_params = cv::Mat());
	/**
	   Constructs a Camera that will open the camera with the given ID and have
	   the given name and description.
	   @param filename The file to open and
	   read a video feed from. This may be the name of a video file, or a URI of
	   a video stream. This will be passed to the underlying OpenCV VideoCapture
	   object, so anything supported by VideoCapture is supported here.

	   @param name The name of the camera. This should ideally be unique, but
	   isn't enforced at this time.

	   @param description An optional description of the camera.
	*/
	Camera(int camera_id, std::string name, std::string description = "",
		   CameraParams intrinsic_params = CameraParams(),
		   cv::Mat extrinsic_params = cv::Mat());
	/**
	   @brief Copy constructor.

	   A Camera object created through this constructor will share access to the
	   underlying camera with the Camera object it is copying. When all Camera
	   objects sharing access are destroyed, the camera will be closed.
	 */
	Camera(const Camera &other);
	/**
	   Returns true if the camera is open.
	 */
	bool isOpen() const;
	/**
	   Returns true if the Camera object has a frame that is different from the
	   last one the client retrieved.

	   @param last_frame_num The frame number that was returned when retrieving
	   a frame.

	   @returns true if there is a new frame (i.e. the current frame number is
	   different than the given one) and false if not.
	 */
	bool hasNext(uint32_t last_frame_num) const;
	/**
	   Retrieves the next frame.

	   @param[out] frame An output parameter for the frame. Whatever is passed
	   in will be overwritten so you do not need to worry about the passed-in
	   Mat having the correct size or format.

	   @param[out] frame_num An output parameter for the frame number.

	   @returns true on success, false if some error occurs.
	 */
	bool next(cv::Mat &frame, uint32_t &frame_num) const;
	bool hasIntrinsicParams() const;
	bool hasExtrinsicParams() const;
	CameraParams getIntrinsicParams() const;
	cv::Mat getExtrinsicParams() const;
	std::string getDescription() const;
	std::string getName() const;
};
/** @} */
} // namespace cam
