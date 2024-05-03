#include "Constants.h"

#include "world_interface/world_interface.h"

namespace Constants {
// TODO: make sure these are still accurate with the new arm.
const double SHOULDER_LENGTH = 0.6; // placeholder(m)
const double ELBOW_LENGTH = 0.7;	// placeholder(m)

/**
   Number of millidegrees per degree
 */
const float MDEG_PER_DEG = 1000;

// TODO: tune these drive constants
const double ROBOT_LENGTH = 1.0;
/**
   Distance between left and right wheels.
*/
const double WHEEL_BASE = 2. / 3.;
/**
  Effective distance between wheels. Tweaked so that actual rover angular rate roughly matches
  the commanded angular rate.
 */
const double EFF_WHEEL_BASE = 1.40;		 // tweaked to match 2-wheel kinematic model
const double WHEEL_RADIUS = 0.15;		 // eyeballed
const double PWM_PER_RAD_PER_SEC = 5000; // eyeballed
const double MAX_DRIVE_PWM = 25000; // can be up to 2^15 - 1 (32767) but we have limited it
									// because driving at full speed draws a massive amount
									// of current that's too much for our power supply (and
									// the rover's full speed is somewhat hard to control
									// while it's driving)
/**
   @brief Maximum tangential velocity for the rover's wheels.

   If the rover is driving straight and not turning, this is the maximum forward velocity
   (i.e. `dx` in setCmdVel()) of the rover.
 */
const double MAX_WHEEL_VEL = WHEEL_RADIUS * MAX_DRIVE_PWM / PWM_PER_RAD_PER_SEC;
/**
   @brief Maximum angular velocity (i.e. `dtheta` in setCmdVel()) of the rover.

   Computed assuming that the left wheel is going at full speed backwards while the right wheel
   is going at full speed forwards.
 */
const double MAX_DTHETA = kinematics::DiffDriveKinematics(EFF_WHEEL_BASE)
							  .wheelVelToRobotVel(-MAX_WHEEL_VEL, MAX_WHEEL_VEL)(2);

namespace Drive {

namespace {
constexpr std::array<int32_t, 4> NORMAL_WHEEL_ROTS = {0, 0, 0, 0};
const swervewheelvel_t TURN_IN_PLACE_VEL =
	robot::swerveKinematics().robotVelToWheelVel(0, 0, 1);
const std::array<int32_t, 4> TURN_IN_PLACE_WHEEL_ROTS = {
	static_cast<int32_t>(TURN_IN_PLACE_VEL.lfRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.rfRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.lbRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.rbRot * 1000)};
constexpr std::array<int32_t, 4> CRAB_WHEEL_ROTS = {90000, 90000, 90000, 90000};
} // namespace

const extern std::unordered_map<DriveMode, std::array<int32_t, 4>> WHEEL_ROTS = {
	{DriveMode::Normal, NORMAL_WHEEL_ROTS},
	{DriveMode::TurnInPlace, TURN_IN_PLACE_WHEEL_ROTS},
	{DriveMode::Crab, CRAB_WHEEL_ROTS}};
} // namespace Drive

// TODO: We need to recalibrate the camera, since we replaced it with a different one.
// TODO: rename cameras (in MC as well) as appropriate
const char* MAST_CAMERA_CONFIG_PATH = "../camera-config/MastCameraCalibration.yml";
const robot::types::CameraID MAST_CAMERA_ID =
	"upperArm"; // TODO: replace with real camera name

const char* FOREARM_CAMERA_CONFIG_PATH = "../camera-config/ForearmCameraCalibration.yml";
const robot::types::CameraID FOREARM_CAMERA_ID = "rear";

const char* HAND_CAMERA_CONFIG_PATH = "../camera-config/HandCameraCalibration.yml";
const robot::types::CameraID HAND_CAMERA_ID = "front";

/**
   @deprecated No need for this constant once we fully switch over the Mission Control PlanViz
 */
const uint16_t PLANVIZ_SERVER_PORT = 9002;
const uint16_t WS_SERVER_PORT = 3001;

/**
   WebSocket server endpoint for the mission control protocol.
 */
const char* MC_PROTOCOL_NAME = "/mission-control";
/**
   WebSocket server endpoint for the simulator protocol.
 */
const char* SIM_PROTOCOL_NAME = "/simulator";
/**
   WebSocket server endpoint for the DGPS protocol.
 */
const char* DGPS_PROTOCOL_NAME = "/dgps";
/**
   Websocket server endpoint for ArduPilot protocol
 */
const char* ARDUPILOT_PROTOCOL_NAME = "/ardupilot";

const std::chrono::milliseconds JOINT_POWER_REPEAT_PERIOD(333);
const std::chrono::milliseconds ARM_IK_UPDATE_PERIOD(50);

namespace Nav {
// Distance (m) we could have traveled forward in the time it takes to turn 1 radian
const double RADIAN_COST = EFF_WHEEL_BASE / 2.0;
// Planner stays this far away from obstacles (m)
const double SAFE_RADIUS = Constants::ROBOT_LENGTH * 1.3;
const int MAX_ITERS = 3000; // Max number of nodes expanded during A* search
const double PLAN_RESOLUTION = Constants::ROBOT_LENGTH; // m
const double SEARCH_RADIUS_INCREMENT = Constants::ROBOT_LENGTH * 3;
const double GPS_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.5;
const double LANDMARK_WAYPOINT_RADIUS = Constants::ROBOT_LENGTH * 1.3;
const double EPS = 2.0; // heuristic weight for weighted A*
} // namespace Nav

// Lidar
namespace Lidar {
const std::string RP_PATH = "/dev/ttyUSB0";
const double MM_PER_M = 1000;
const uint32_t RPLIDAR_A1_BAUDRATE = 115200;
const uint32_t RPLIDAR_S1_BAUDRATE = 256000;
} // namespace Lidar

// Video encoding
namespace video {
const int H264_RF_CONSTANT = 40;
const std::unordered_map<robot::types::CameraID, int> STREAM_RFS = {
	{Constants::HAND_CAMERA_ID, 30}};
} // namespace video

// Arm inverse kinematics
namespace arm {
/**
 * Percentage of fully extended overall arm length to limit arm extension within.
 */
const double SAFETY_FACTOR = 0.95;

/**
 * Maximum commanded end-effector velocity, in m/s
 */
const double MAX_EE_VEL = 0.1;
const double IK_SOLVER_THRESH = 0.001;

const int IK_SOLVER_MAX_ITER = 50;

/**
 * The joints corresponding to the motors used for IK in the arm. The ordering in this array is
 * the canonical ordering of these joints for IK purposes.
 */
const std::array<robot::types::jointid_t, 2> IK_MOTOR_JOINTS = {
	robot::types::jointid_t::shoulder, robot::types::jointid_t::elbow};

/**
 * The motors used in IK. The i-th element in this array corresponds to the joint in the i-th
 * element of `IK_MOTOR_JOINTS`
 */
const std::array<robot::types::motorid_t, 2> IK_MOTORS = ([]() {
	std::array<robot::types::motorid_t, IK_MOTOR_JOINTS.size()> ret{};
	for (size_t i = 0; i < IK_MOTOR_JOINTS.size(); i++) {
		ret[i] = JOINT_MOTOR_MAP.at(IK_MOTOR_JOINTS[i]);
	}
	return ret;
})();
} // namespace arm

const double CONTROL_HZ = 10.0;
} // namespace Constants
