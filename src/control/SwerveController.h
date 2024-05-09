// Move kinematics out of world interface
// Own kinematics
// Tell it the drive mode
// Owns drive mode (no more global)
// Ask it different drive commands
// SetTurnInPlace
// Do corresponding checks if not overriden
// This would still call the world interface method
// Set motor power directly here
// Remove crab and turn in place
// Create periodic task to set power velocities
#pragma once

#include "../Constants.h"
#include "../Globals.h"
#include "../kinematics/DiffDriveKinematics.h"
#include "../kinematics/SwerveDriveKinematics.h"

#include <map>

namespace control {
enum class DriveMode {
	Normal,
	TurnInPlace,
	Crab,
};

static const std::map<DriveMode, std::string> driveModeStrings = {
	{DriveMode::Normal, "Normal"},
	{DriveMode::TurnInPlace, "TurnInPlace"},
	{DriveMode::Crab, "Crab"}};

class SwerveController {
public:
	SwerveController(){};

	double setTurnInPlaceCmdVel(double dtheta);

	double setCrabCmdVel(double dtheta, double dy);

	static const kinematics::SwerveDriveKinematics& swerveKinematics();

	/**
	 * @brief Check to see if all wheels are at their target position.
	 *
	 * @param mode The drive mode to compare wheel rotations against.
	 * @return bool if the wheels are within `Constants::Drive::STEER_EPSILON` of their their
	 * target position.
	 */
	bool checkWheelRotation(DriveMode mode);
};
} // namespace control