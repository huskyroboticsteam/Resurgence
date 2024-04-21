#include "Constants.h"

#include "./world_interface/world_interface.h"
#include "Globals.h"

namespace Constants {
namespace Drive {
constexpr std::array<int32_t, 4> NORMAL_WHEEL_ROTS = {0, 0, 0, 0};
const swervewheelvel_t TURN_IN_PLACE_VEL =
	robot::swerveKinematics().robotVelToWheelVel(0, 0, 1);
const std::array<int32_t, 4> TURN_IN_PLACE_WHEEL_ROTS = {
	static_cast<int32_t>(TURN_IN_PLACE_VEL.lfRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.rfRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.lbRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.rbRot * 1000)};
constexpr std::array<int32_t, 4> CRAB_WHEEL_ROTS = {90000, 90000, 90000, 90000};
const extern std::unordered_map<DriveMode, std::array<int32_t, 4>> WHEEL_ROTS = {
	{DriveMode::Normal, NORMAL_WHEEL_ROTS},
	{DriveMode::TurnInPlace, TURN_IN_PLACE_WHEEL_ROTS},
	{DriveMode::Crab, CRAB_WHEEL_ROTS}};
} // namespace Drive
} // namespace Constants