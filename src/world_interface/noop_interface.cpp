#include "world_interface.h"

namespace robot {
extern const WorldInterface WORLD_INTERFACE = WorldInterface::noop;

namespace {
bool is_emergency_stopped = false;
}

void world_interface_init(std::optional<std::reference_wrapper<net::websocket::SingleClientWSServer>> wsServer, bool initOnlyMotors) {}

void emergencyStop() { is_emergency_stopped = true; }
bool isEmergencyStopped() { return is_emergency_stopped; }

std::shared_ptr<robot::base_motor> getMotor(robot::types::motorid_t motor) { return nullptr; }
void setMotorPower(robot::types::motorid_t motor, double power) {}
void setMotorPos(robot::types::motorid_t motor, int32_t targetPos) {}
void setMotorVel(robot::types::motorid_t motor, int32_t targetVel) {}
types::DataPoint<int32_t> getMotorPos(robot::types::motorid_t motor) { return {}; }

void setIndicator(types::indication_t signal) {}
void setServoPos(robot::types::servoid_t servo, int32_t position) {}
void setRequestedStepperTurnAngle(robot::types::stepperid_t stepper, int16_t angle) {}
void setActuator(uint8_t value) {}

callbackid_t addLimitSwitchCallback(
	robot::types::motorid_t motor,
	const std::function<void(
		robot::types::motorid_t motor,
		robot::types::DataPoint<robot::types::LimitSwitchData> limitSwitchData)>& callback) { return 0; }
void removeLimitSwitchCallback(callbackid_t id) {}

std::unordered_set<types::CameraID> getCameras() { return {}; }
std::shared_ptr<types::CameraHandle> openCamera(types::CameraID camID) { return nullptr; };
bool hasNewCameraFrame(types::CameraID camera, uint32_t oldFrameNum) { return false; }
types::DataPoint<types::CameraFrame> readCamera(types::CameraID camera) { return {}; }
std::optional<cam::CameraParams> getCameraIntrinsicParams(types::CameraID camera) { return {}; }
std::optional<cv::Mat> getCameraExtrinsicParams(types::CameraID camera) { return {}; }

types::landmarks_t readLandmarks() { return {}; }

// bool gpsHasFix() { return false; }
// types::DataPoint<navtypes::point_t> readGPS() { return {}; }
// types::DataPoint<double> readIMUHeading() { return {}; }
types::DataPoint<Eigen::Quaterniond> readIMU() { return {}; }
types::DataPoint<navtypes::pose_t> getTruePose() { return {}; }

// namespace gps {
// robot::types::DataPoint<navtypes::gpscoords_t> readGPSCoords() { return {}; }
// }
}