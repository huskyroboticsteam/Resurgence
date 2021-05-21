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

   Cameras can be defined in YAML files (see @ref cameraconfig) and opened using
   these files at runtime (see Camera::openFromConfigFile()).
 */

/**
   @namespace cam
   @brief Namespace for camera access system.
 */
namespace cam
{

/**
   @addtogroup camera
   @{
 */

/**
	@name Configuration File Keys

	The following are constants for the keys used in the camera configuration
	files. See @ref cameraconfig for more details.
*/
/**@{*/
/**
   Config file key for camera filename.
 */
const std::string KEY_FILENAME = "filename";
/**
   Config file key for camera id.
 */
const std::string KEY_CAMERA_ID = "camera_id";
/**
   Config file key for intrinsic parameters.
 */
const std::string KEY_INTRINSIC_PARAMS = "intrinsic_params";
/**
   Config file key for extrinsic parameters.
 */
const std::string KEY_EXTRINSIC_PARAMS = "extrinsic_params";
/**
   Config file key for calibration information.
 */
const std::string KEY_CALIB_INFO = "calib_info";
/**
   Config file key for camera name.
 */
const std::string KEY_NAME = "name";
/**
   Config file key for camera description.
 */
const std::string KEY_DESCRIPTION = "description";
/**@}*/

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
class Camera
{
private:
	std::shared_ptr<cv::Mat> _frame;
	std::shared_ptr<uint32_t> _frame_num;
	std::shared_ptr<cv::VideoCapture> _capture;
	std::string _name;
	std::string _description;
	std::shared_ptr<std::mutex> _frame_lock;
	std::shared_ptr<std::mutex> _capture_lock;
	std::shared_ptr<std::thread> _thread;
	CameraParams _intrinsic_params;
	cv::Mat _extrinsic_params;
	std::shared_ptr<bool> _running;
	void captureLoop();
	void init(const cv::Mat &extrinsic_params);

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
	   @brief Opens a Camera that is not already open.

	   Will open the given file for input and use the given intrinsic and
	   extrinsic parameters, similar to the constructor.

	   @returns true if opening the camera succeeds, and false if not.
	 */
	bool open(std::string filename, CameraParams intrinsic_params = CameraParams(),
			  cv::Mat extrinsic_params = cv::Mat());
	/**
	   @brief Opens a Camera that is not already open.

	   Will open the camera at the given camera ID and use the given intrinsic
	   and extrinsic parameters, similar to the constructor.

	   @returns true if opening the camera succeeds, and false if not.
	*/
	bool open(int camera_id, CameraParams intrinsic_params = CameraParams(),
			  cv::Mat extrinsic_params = cv::Mat());
	/**
	   @brief Constructs a Camera that will open the given file for input and
	   have the given name and description.

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
	   @brief Constructs a Camera that will open the camera with the given ID
	   and have the given name and description.

	   @param filename The file to open and read a video feed from. This may be
	   the name of a video file, or a URI of a video stream. This will be passed
	   to the underlying OpenCV VideoCapture object, so anything supported by
	   VideoCapture is supported here.

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
	   @brief Opens the camera using the given configuration file.

	   @param filename The path to the configuration file to open and
	   read. Configuration file should be formatted as described in @ref
	   cameraconfig.

	   @throws invalid_camera_config If the configuration is invalid for any
	   reason.
	*/
	bool openFromConfigFile(std::string filename);

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
	bool next(cv::Mat &frame, uint32_t &frame_num) const;
	/**
	   @brief Returns true if this camera has associated intrinsic parameters.
	 */
	bool hasIntrinsicParams() const;
	/**
	   @brief Returns true if this camera has associated extrinsic parameters.
	 */
	bool hasExtrinsicParams() const;
	/**
	   @brief Returns the associated intrinsic parameters as a CameraParams
	   object.
	 */
	CameraParams getIntrinsicParams() const;
	/**
	   @brief Returns the associated extrinsic parameters as an OpenCV matrix.
	 */
	cv::Mat getExtrinsicParams() const;
	/**
	   @brief Returns the description of the camera.

	   Note the description is optional, in which case an empty string will be
	   returned.
	 */
	std::string getDescription() const;
	/**
	   @brief Returns the name of the camera.
	 */
	std::string getName() const;
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

/**
   Exception for errors in the camera configuration.
 */
class invalid_camera_config : public std::exception
{
public:
	/**
	   Constructs an invalid_camera_config exception with the default message "Invalid camera
	   configuration".
	 */
	invalid_camera_config();
	/**
	   Constructs an invalid_camera_config exception with the given message appended to
	   "Invalid camera configuration:".
	   @param msg The message to use for the exception.
	 */
	invalid_camera_config(const std::string &msg);
	/**
	   Returns the exception message as a C string.
	 */
	const char *what() const noexcept;

private:
	std::string _msg;
};

/** @} */
} // namespace cam
