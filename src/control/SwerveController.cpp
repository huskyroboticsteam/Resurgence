#include "SwerveController.h"

#include "../Constants.h"

using namespace control;
using robot::types::motorid_t;

control::SwerveController::SwerveController(double baseWidth, double baseLength,
											int epsilon = 1000)
	: driveMode(DriveMode::Normal), swerve_kinematics(baseWidth, baseLength),
	  crab_kinematics(baseLength), steer_epsilon(epsilon) {}

drive_commands_t
SwerveController::setTurnInPlaceCmdVel(double dtheta, const swerve_rots_t& wheel_rots) const {
	double dummyScaleFactor;
	drive_commands_t commands = setTurnInPlaceCmdVel(dtheta, wheel_rots, dummyScaleFactor);
	return commands;
}

drive_commands_t SwerveController::setTurnInPlaceCmdVel(double dtheta,
														const swerve_rots_t& wheel_rots,
														double& scaleFactor) const {
	if (!checkWheelRotation(DriveMode::TurnInPlace, wheel_rots)) {
		scaleFactor = 0;
		return {0.0, 0.0, 0.0, 0.0};
	}

	kinematics::swervewheelvel_t wheelVels =
		swerve_kinematics.robotVelToWheelVel(0, 0, dtheta);
	double lfPower = wheelVels.lfVel / Constants::MAX_WHEEL_VEL;
	double lbPower = wheelVels.lbVel / Constants::MAX_WHEEL_VEL;
	double rfPower = wheelVels.rfVel / Constants::MAX_WHEEL_VEL;
	double rbPower = wheelVels.rbVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::max(std::abs(lfPower), std::abs(lbPower)),
								std::max(std::abs(rfPower), std::abs(rbPower)));

	if (maxAbsPWM > 1) {
		lfPower /= maxAbsPWM;
		lbPower /= maxAbsPWM;
		rfPower /= maxAbsPWM;
		rbPower /= maxAbsPWM;
		scaleFactor = maxAbsPWM;
	} else {
		scaleFactor = 1.0;
	}

	return {lfPower, rfPower, lbPower, rbPower};
}

drive_commands_t SwerveController::setCrabCmdVel(double dtheta, double dy,
												 const swerve_rots_t& wheel_rots) const {
	double dummyScaleFactor;
	drive_commands_t commands = setCrabCmdVel(dtheta, dy, wheel_rots, dummyScaleFactor);
	return commands;
}

drive_commands_t SwerveController::setCrabCmdVel(double dtheta, double dy,
												 const swerve_rots_t& wheel_rots,
												 double& scaleFactor) const {
	if (!checkWheelRotation(DriveMode::Crab, wheel_rots)) {
		scaleFactor = 0;
		return {0.0, 0.0, 0.0, 0.0};
	}

	kinematics::wheelvel_t wheelVels = crab_kinematics.robotVelToWheelVel(dy, dtheta);
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
		scaleFactor = maxAbsPWM;
	} else {
		scaleFactor = 1.0;
	}

	// Return order is lf, rf, lb, rb
	// We rotate the output reference frame by 90 degrees CCW for crab, so right outputs are
	// sent to the front and left outputs are sent to the back
	return {rPWM, rPWM, lPWM, lPWM};
}

bool SwerveController::checkWheelRotation(DriveMode mode,
										  const swerve_rots_t& wheel_rots) const {
	std::array<int, 4> rots = {wheel_rots.lfRot, wheel_rots.rfRot, wheel_rots.lbRot,
							   wheel_rots.rbRot};
	swerve_rots_t target = getSteerRots(mode);
	std::array<int, 4> target_rots = {target.lfRot, target.rfRot, target.lbRot, target.rbRot};
	// Immediately return true if steer checking is overridden
	if (override_steer_check) {
		return true;
	}

	for (int i = 0; i < 4; i++) {
		if (std::abs(rots[i] - target_rots[i]) > steer_epsilon)
			return false;
	}
	return true;
}

swerve_rots_t SwerveController::getSteerRots(DriveMode mode) const {
	switch (mode) {
		case DriveMode::Normal:
			return {0, 0, 0, 0};
		case DriveMode::TurnInPlace: {
			kinematics::swervewheelvel_t wheelVels =
				swerve_kinematics.robotVelToWheelVel(0, 0, 1);
			return {static_cast<int32_t>(wheelVels.lfRot * 1000),
					static_cast<int32_t>(wheelVels.rfRot * 1000),
					static_cast<int32_t>(wheelVels.lbRot * 1000),
					static_cast<int32_t>(wheelVels.rbRot * 1000)};
		}
		case DriveMode::Crab:
			return {90000, 90000, 90000, 90000};
		default:
			return {0, 0, 0, 0}; // Should never happen
	}
}

const kinematics::SwerveDriveKinematics& SwerveController::swerveKinematics() const {
	return swerve_kinematics;
}

DriveMode SwerveController::getDriveMode() const {
	return driveMode;
}

swerve_rots_t SwerveController::setDriveMode(DriveMode mode) {
	driveMode = mode;
	return getSteerRots(mode);
}

void SwerveController::setOverride(bool override) {
	override_steer_check = override;
}

namespace util {
std::string to_string(control::DriveMode mode) {
	using control::DriveMode;

	switch (mode) {
		case DriveMode::Normal:
			return "Normal";
		case DriveMode::TurnInPlace:
			return "TurnInPlace";
		case DriveMode::Crab:
			return "Crab";
		default:
			return "<unknown>";
	}
}
} // namespace util