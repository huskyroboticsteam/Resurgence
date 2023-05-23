#include "Globals.h"

#include "Constants.h"
#include "world_interface/data.h"

#include <atomic>
#include <vector>

namespace Globals {
RoverState curr_state;
net::websocket::SingleClientWSServer websocketServer("DefaultServer",
													 Constants::WS_SERVER_PORT);
std::atomic<bool> E_STOP = false;
std::atomic<bool> AUTONOMOUS = false;
robot::types::mountedperipheral_t mountedPeripheral = robot::types::mountedperipheral_t::none;
kinematics::PlanarArmKinematics<2> planarArmKinematics =
	kinematics::PlanarArmKinematics<2>({Constants::arm::SEGMENT_LENGTHS.at("shoulder"),
										Constants::arm::SEGMENT_LENGTHS.at("elbow")},
									   {Constants::arm::JOINT_LIMITS.at("shoulder").first,
										Constants::arm::JOINT_LIMITS.at("elbow").first},
									   {Constants::arm::JOINT_LIMITS.at("shoulder").second,
										Constants::arm::JOINT_LIMITS.at("elbow").second},
									   0.0, 0);
control::PlanarArmController<2> planarArmController =
	control::PlanarArmController<2>{{0, 0}, planarArmKinematics};
} // namespace Globals
