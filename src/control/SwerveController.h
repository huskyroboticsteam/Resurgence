#pragma once

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
	SwerveController(double baseWidth, double baseLength)
		: driveMode(DriveMode::Normal, false), swerve_kinematics(baseWidth, baseLength),
		  crab_kinematics(baseLength){};

	/**
	 * @brief Request the robot to turn in place using swerve.
	 *
	 * @param dtheta The heading velocity.
	 * @param wheel_rots The rotation of all four wheels in order FL, FR, RL, RR
	 * @return double If the requested velocities are too high, they will be scaled down.
	 * The returned value is the scale divisor. If no scaling was performed, 1 is returned.
	 */
	std::pair<double, std::vector<double>> setTurnInPlaceCmdVel(double dtheta,
																std::vector<int> wheels);
	/**
	 * @brief Request the robot to drive side to side and turn with given velocities using
	 * swerve. Essentially rotate the reference frame of the robot by 90 degrees.
	 *
	 * @param dtheta The heading velocity.
	 * @param dy The side-to-side velocity.
	 * @param wheel_rots The rotation of all four wheels in order FL, FR, RL, RR
	 * @return double If the requested velocities are too high, they will be scaled down.
	 * The returned value is the scale divisor. If no scaling was performed, 1 is returned.
	 */
	std::pair<double, std::vector<double>> setCrabCmdVel(double dtheta, double dy,
														 std::vector<int> wheels);

	kinematics::SwerveDriveKinematics swerveKinematics();

	/**
	 * @brief Check to see if all wheels are at their target position.
	 *
	 * @param mode The drive mode to compare wheel rotations against.
	 * @param wheel_rots The rotation of all four wheels in order FL, FR, RL, RR
	 * @return bool if the wheels are within `Constants::Drive::STEER_EPSILON` of their their
	 * target position.
	 */
	bool checkWheelRotation(DriveMode mode, std::vector<int> wheel_rots);

	// The boolean here is whether or not wheel rotation check should be ignored when doing
	// swerve-based driving
	std::pair<DriveMode, bool> driveMode;

private:
	const kinematics::SwerveDriveKinematics swerve_kinematics;
	const kinematics::DiffDriveKinematics crab_kinematics;
};
} // namespace control