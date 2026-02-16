#include "Constants.h"

#include "kinematics/DiffDriveKinematics.h"

namespace Constants {

namespace Arm {
const double SHOULDER_LENGTH = 0.6;
const double ELBOW_LENGTH = 0.7;
const double SAFETY_FACTOR = 0.95;
const double MAX_EE_VEL = 0.1;
const double IK_SOLVER_THRESH = 0.001;
const int IK_SOLVER_MAX_ITER = 50;

const std::array<robot::types::jointid_t, 2> IK_MOTOR_JOINTS = {
	robot::types::jointid_t::shoulder, robot::types::jointid_t::elbow};

const std::array<robot::types::motorid_t, 2> IK_MOTORS = ([]() {
	std::array<robot::types::motorid_t, IK_MOTOR_JOINTS.size()> ret{};
	for (size_t i = 0; i < IK_MOTOR_JOINTS.size(); i++) {
		ret[i] = JOINT_MOTOR_MAP.at(IK_MOTOR_JOINTS[i]);
	}
	return ret;
})();

} // namespace Arm

namespace Drive {
const double ROBOT_LENGTH = 1.0;
const double ROBOT_WIDTH = 1.0;
const double WHEEL_BASE = 2. / 3.;
const double EFF_WHEEL_BASE = 1.40;
const double WHEEL_RADIUS = 0.15;
const double PWM_PER_RAD_PER_SEC = 5000;
const double MAX_DRIVE_PWM = 25000;

const double MAX_WHEEL_VEL = WHEEL_RADIUS * MAX_DRIVE_PWM / PWM_PER_RAD_PER_SEC;

const double MAX_DTHETA = kinematics::DiffDriveKinematics(EFF_WHEEL_BASE)
							  .wheelVelToRobotVel(-MAX_WHEEL_VEL, MAX_WHEEL_VEL)(2);

const double STEER_EPSILON = 10000;

} // namespace Drive

namespace Camera {
const robot::types::CameraID HAND_CAMERA_ID = "hand";
const robot::types::CameraID WRIST_CAMERA_ID = "wrist";
const robot::types::CameraID MAST_CAMERA_ID = "mast";

const std::unordered_set<robot::types::CameraID> CAMERA_SET = {
   HAND_CAMERA_ID,
   WRIST_CAMERA_ID,
   MAST_CAMERA_ID
};

const std::unordered_map<robot::types::CameraID, std::string> CAMERA_CONFIG_PATHS = {
	{HAND_CAMERA_ID, "../camera-config/HandCameraCalibration.yml"},
	{WRIST_CAMERA_ID, "../camera-config/WristCameraCalibration.yml"},
	{MAST_CAMERA_ID, "../camera-config/MastCameraCalibration.yml"},
};

const int H264_RF_CONSTANT = 40;
const std::unordered_map<robot::types::CameraID, int> STREAM_RFS = {
	{HAND_CAMERA_ID, 30}};

} // namespace Camera

namespace Network {
const uint16_t WS_SERVER_PORT = 3001;

const char* MC_PROTOCOL_NAME = "/mission-control";
const char* SIM_PROTOCOL_NAME = "/simulator";
const char* ARDUPILOT_PROTOCOL_NAME = "/ardupilot";

} // namespace Network

namespace Nav {
using namespace Constants::Drive;
const double RADIAN_COST = EFF_WHEEL_BASE / 2.0;
const double SAFE_RADIUS = ROBOT_LENGTH * 1.3;
const int MAX_ITERS = 3000;
const double PLAN_RESOLUTION = ROBOT_LENGTH;
const double SEARCH_RADIUS_INCREMENT = ROBOT_LENGTH * 3;
const double GPS_WAYPOINT_RADIUS = ROBOT_LENGTH * 1.5;
const double LANDMARK_WAYPOINT_RADIUS = ROBOT_LENGTH * 1.3;
const double EPS = 2.0;

const double THETA_KP = 2.0;
const double DRIVE_VEL = 1.5;
const double SLOW_DRIVE_THRESHOLD = 8.0;
const double DONE_THRESHOLD = 3.0;
const util::dseconds CLOSE_TO_TARGET_DUR_VAL = std::chrono::milliseconds(750);

} // namespace Nav

const double CONTROL_HZ = 10.0;

const std::chrono::milliseconds JOINT_POWER_REPEAT_PERIOD(333);
const std::chrono::milliseconds ARM_IK_UPDATE_PERIOD(50);

} // namespace Constants
