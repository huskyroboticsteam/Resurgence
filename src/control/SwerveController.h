#pragma once

#include "../kinematics/DiffDriveKinematics.h"
#include "../kinematics/SwerveDriveKinematics.h"

#include <map>

namespace control {
/**
 * @brief The current drive mode of the rover, which determines the rotation of individual
 * swerve wheels and how drive requests should be handled
 */
enum class DriveMode {
	/**
	 * @brief Standard drive mode, supporting either Tank or standard differential steering
	 * mode
	 */
	Normal,

	/**
	 * @brief Turn-in-place drive mode where the rover rotates around its center without
	 * translation
	 */
	TurnInPlace,

	/**
	 * @brief Crab drive mode where the rover moves sideways with optional rotation. Just like
	 * Normal, just with a reference frame rotated 90 degrees CCW
	 */
	Crab,
};

/**
 * @brief Represents the rotations of all four steer motors in in millidegrees.
 */
struct swerve_rots_t {
	int lfRot;
	int rfRot;
	int lbRot;
	int rbRot;
};

/**
 * @brief The power commands to all four steer motors.
 */
struct swerve_commands_t {
	double lfPower;
	double rfPower;
	double lbPower;
	double rbPower;
};

/**
 * @brief Controller to handle the wheel rotations for driving with swerve modes
 */
class SwerveController {
public:
	/**
	 * @brief Construct a new controller object.
	 *
	 * @param baseWidth The width of the wheelbase. The units themselves don't matter, as
	 * long as you're consistent.
	 * @param baseLength The length of the wheelbase. The units themselves don't matter, as
	 * long as you're consistent.
	 * @param epsilon The allowable error from the actual wheel rotation to the wheel
	 * rotations defined by `DRIVE_MODE` to process a drive request
	 */
	SwerveController(double baseWidth, double baseLength, int epsilon);

	/**
	 * @brief Request the robot to turn in place using swerve.
	 *
	 * @param dtheta The heading velocity.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @return swerve_commands_t The power commands for all four wheels.
	 */
	swerve_commands_t setTurnInPlaceCmdVel(double dtheta,
										   const swerve_rots_t& wheel_rots) const;

	/**
	 * @brief Request the robot to turn in place using swerve.
	 *
	 * @param dtheta The heading velocity.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @param scaleFactor This is an output parameter. If the requested velocities
	 * are too high, they will be scaled down. The returned value is the scale divisor. If no
	 * scaling was performed, 1 is returned.
	 * @return swerve_commands_t The power commands for all four wheels.
	 */
	swerve_commands_t setTurnInPlaceCmdVel(double dtheta, const swerve_rots_t& wheel_rots,
										   double& scaleFactor) const;

	/**
	 * @brief Request the robot to drive side to side and turn with given velocities using
	 * swerve. Essentially rotate the reference frame of the robot by 90 degrees CCW.
	 *
	 * @param dtheta The heading velocity.
	 * @param dy The side-to-side velocity.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @return swerve_commands_t The power commands for all four wheels.
	 */
	swerve_commands_t setCrabCmdVel(double dtheta, double dy,
									const swerve_rots_t& wheel_rots) const;

	/**
	 * @brief Request the robot to drive side to side and turn with given velocities using
	 * swerve. Essentially rotate the reference frame of the robot by 90 degrees CCW.
	 *
	 * @param dtheta The heading velocity.
	 * @param dy The side-to-side velocity.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @param scaleFactor This is an output parameter. If the requested velocities
	 * are too high, they will be scaled down. The returned value is the scale divisor. If no
	 * scaling was performed, 1 is returned.
	 * @return swerve_commands_t The power commands for all four wheels.
	 */
	swerve_commands_t setCrabCmdVel(double dtheta, double dy, const swerve_rots_t& wheel_rots,
									double& scaleFactor) const;

	/**
	 * @brief Get the SwerveDriveKinematics object used by the controller.
	 *
	 * @return const kinematics::SwerveDriveKinematics& The SwerveDriveKinematics object.
	 */
	const kinematics::SwerveDriveKinematics& swerveKinematics() const;

	/**
	 * @brief Set the override flag for wheel rotation checking.
	 *
	 * When set to true, the checkWheelRotation function will always return true,
	 * bypassing the wheel rotation check.
	 *
	 * @param override Override the wheel rotation check.
	 */
	void setOverride(bool override);

	/**
	 * @brief Get the current drive mode of the controller.
	 *
	 * @return DriveMode The current drive mode.
	 */
	DriveMode getDriveMode() const;

	/**
	 * @brief Set the drive mode of the controller.
	 *
	 * @param mode The new drive mode to set.
	 * @return swerve_rots_t The wheel steer rotations associated with this drive mode.
	 */
	swerve_rots_t setDriveMode(DriveMode mode);

	/**
	 * @brief Check to see if all wheels are at their target position.
	 *
	 * @param mode The drive mode to compare wheel rotations against.
	 * @param wheel_rots The rotation of all four wheels in millidegrees
	 * @return bool if the wheels are within `STEER_EPSILON` of their
	 * target position OR if `overide_steer_check` is true.
	 */
	bool checkWheelRotation(DriveMode mode, const swerve_rots_t& wheel_rots) const;

	/**
	 * @brief Get the wheel steer rotations for a given drive mode in millidegrees.
	 */
	swerve_rots_t getSteerRots(DriveMode mode) const;

private:
	const kinematics::SwerveDriveKinematics swerve_kinematics;
	const kinematics::DiffDriveKinematics crab_kinematics;
	const double steer_epsilon;

	/**
	 * @brief The current drive mode.
	 */
	DriveMode driveMode;

	/**
	 * @brief Whether or not wheel rotation check should be ignored when
	 * doing swerve-based driving.
	 */
	bool override_steer_check;
};
} // namespace control

namespace util {
std::string to_string(control::DriveMode mode);
} // namespace util
