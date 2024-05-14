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

// Represents the rotations of all four steer motors in in millidegrees.
struct swerve_rots_t {
	int lfRot;
	int rfRot;
	int lbRot;
	int rbRot;
};

// The PWM commands to all four steer motors.
struct swerve_commands_t {
	double lfPWM;
	double rfPWM;
	double lbPWM;
	double rbPWM;
};

class SwerveController {
public:
	SwerveController(double baseWidth, double baseLength);

	/**
	 * @brief Request the robot to turn in place using swerve.
	 *
	 * @param dtheta The heading velocity.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @return std::pair<double, swerve_commands_t> First: If the requested velocities are too
	 * high, they will be scaled down. The returned value is the scale divisor. If no scaling
	 * was performed, 1 is returned. Second: The PWM commands for all four wheels.
	 */
	std::pair<double, swerve_commands_t> setTurnInPlaceCmdVel(double dtheta,
															  swerve_rots_t wheels);
	/**
	 * @brief Request the robot to drive side to side and turn with given velocities using
	 * swerve. Essentially rotate the reference frame of the robot by 90 degrees.
	 *
	 * @param dtheta The heading velocity.
	 * @param dy The side-to-side velocity.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @return std::pair<double, swerve_commands_t> First: If the requested velocities are too
	 * high, they will be scaled down. The returned value is the scale divisor. If no scaling
	 * was performed, 1 is returned. Second: The PWM commands for all four wheels.
	 */
	std::pair<double, swerve_commands_t> setCrabCmdVel(double dtheta, double dy,
													   swerve_rots_t wheels);

	kinematics::SwerveDriveKinematics swerveKinematics();

	/**
	 * @brief Check to see if all wheels are at their target position.
	 *
	 * @param mode The drive mode to compare wheel rotations against.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @return bool if the wheels are within `Constants::Drive::STEER_EPSILON` of their their
	 * target position.
	 */
	bool checkWheelRotation(DriveMode mode, swerve_rots_t wheel_rots);

	// The boolean here is whether or not wheel rotation check should be ignored when doing
	// swerve-based driving
	std::pair<DriveMode, bool> driveMode;

	const std::unordered_map<DriveMode, std::array<int32_t, 4>> WHEEL_ROTS;

private:
	const kinematics::SwerveDriveKinematics swerve_kinematics;
	const kinematics::DiffDriveKinematics crab_kinematics;
	const double STEER_EPSILON = 1000;
};
} // namespace control