#pragma once

#include "../world_interface/data.h"

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

/**
   @defgroup camera Camera Access
   @brief This module provides an interface for defining and accessing the
   different cameras attached to the rover.

   Cameras can be defined in YAML files (see @ref cameraconfig) and opened using
   these files at runtime (see Camera::openFromConfigFile()).
 */

/**
   @namespace cam
   @brief Namespace for camera access system.
 */
namespace cam {

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

	@warning Note that intrinsic and extrinsic parameters are optional for a
	Camera object; Camera objects that are missing either intrinsic parameters,
	extrinsic parameters, or both may not be suitable for some computer vision
	operations. For example, AR tag detection requires intrinsic parameters and
	cannot be performed without them. Use Camera::hasIntrinsicParams() and
	Camera::hasExtrinsicParams() to check for the presence of each.
 */
class Camera {
private:
	std::shared_ptr<cv::Mat> _frame;
	std::shared_ptr<uint32_t> _frame_num;
	std::shared_ptr<robot::types::datatime_t> _frame_time;
	std::shared_ptr<cv::VideoCapture> _capture;
	std::string _name;
	std::string _description;
	int _width;
	int _height;
	std::shared_ptr<std::mutex> _frame_lock;
	std::shared_ptr<std::mutex> _capture_lock;
	std::shared_ptr<std::thread> _thread;
	std::shared_ptr<bool> _running;
	void captureLoop();
	void init();

public:
	/**
	   @brief Constructs an "empty" Camera.

	   This constructor is included for convenience. Camera objects constructed
	   with this constructor will not actually access any camera, and
	   Camera::isOpen() will return false. The Camera can be replaced later, or
	   manually opened with the Camera::open() method.
	*/
	Camera();
	/**
	   @brief Constructs a Camera that will open the camera with the given ID
	   and have the given name and description.

	   @param camera_id The ID of the Camera to open and read a video feed from.
	*/
	Camera(robot::types::CameraID camera_id);
	/**
	   @brief Copy constructor.

	   A Camera object created through this constructor will share access to the
	   underlying camera with the Camera object it is copying. When all Camera
	   objects sharing access are destroyed, the camera will be closed.
	 */
	Camera(const Camera& other);
	/**
	   @brief Opens a Camera that is not already open.

	   Will open the camera at the given camera ID, similar to the constructor.

	   @returns true if opening the camera succeeds, and false if not.
	*/
	bool open(robot::types::CameraID camera_id);
	/**
		@brief Get camera configs for the specified camera.

		@param camera_id The ID of the Camera to get the configs for.

		@returns The Gstream with configs.
	 */
	std::stringstream GStreamerFromFile(robot::types::CameraID camera_id);
	/**
	   @brief Returns true if the camera is open.
	 */
	bool isOpen() const;
	/**
	   @brief Returns true if the Camera object has a frame that is different
	   from the last one the client retrieved.

	   @param last_frame_num The frame number that was returned when retrieving
	   a frame.

	   @returns true if there is a new frame (i.e. the current frame number is
	   different than the given one) and false if not.
	 */
	bool hasNext(uint32_t last_frame_num) const;
	/**
	   @brief Retrieves the next frame.

	   @param[out] frame An output parameter for the frame. All data will be
	   overwritten so you do not need to worry about the passed-in Mat having
	   the correct size or format.

	   @param[out] frame_num An output parameter for the frame number.

	   @returns true on success, false if some error occurs.
	 */
	bool next(cv::Mat& frame, uint32_t& frame_num) const;
	/**
	 * @brief Retrives the next frame.
	 *
	 * @param[out] frame An output parameter for the frame. All data will be
	 * overwritten so you do not need to worry about the passed-in Mat having
	 * the correct size or format.
	 * @param[out] frame_num An output parameter for the frame number.
	 * @param[out] frame_time An output parameter for the time at which the frame was captured.
	 * Note that this is not the same as the current time.
	 * @returns true on success, false if some error occurs.
	 */
	bool next(cv::Mat& frame, uint32_t& frame_num, robot::types::datatime_t& frame_time) const;
	/**
	   @brief Returns the name of the camera.
	 */
	std::string getName() const;
	/**
	   @brief Returns the description of the camera.

	   Note the description is optional, in which case an empty string will be
	   returned.
	 */
	std::string getDescription() const;
	/**
		@brief Returns the width of the camera feed.
	 */
	int getWidth() const;
	/**
		@brief Returns the height of the camera feed.
	 */
	int getHeight() const;
	/**
	   @brief Updates the name of the camera to the given name.
	   @param new_name The new name for the camera.
	 */
	void setName(std::string new_name);
	/**
	   @brief Updates the description of the camera to the given description.
	   @param new_description The new description for the camera.
	 */
	void setDescription(std::string new_description);
};

/** @} */
} // namespace cam
