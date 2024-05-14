#include "SwerveController.h"

#include "../Constants.h"
#include "../Globals.h"

using namespace control;
using Globals::swerveController;
using robot::types::motorid_t;

namespace {
const std::array<int32_t, 4> NORMAL_WHEEL_ROTS = {0, 0, 0, 0};
const kinematics::swervewheelvel_t TURN_IN_PLACE_VEL =
	swerveController.swerveKinematics().robotVelToWheelVel(0, 0, 1);
const std::array<int32_t, 4> TURN_IN_PLACE_WHEEL_ROTS = {
	static_cast<int32_t>(TURN_IN_PLACE_VEL.lfRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.rfRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.lbRot * 1000),
	static_cast<int32_t>(TURN_IN_PLACE_VEL.rbRot * 1000)};
const std::array<int32_t, 4> CRAB_WHEEL_ROTS = {90000, 90000, 90000, 90000};
} // namespace

control::SwerveController::SwerveController(double baseWidth, double baseLength)
	: driveMode(DriveMode::Normal, false), swerve_kinematics(baseWidth, baseLength),
	  crab_kinematics(baseLength),
	  WHEEL_ROTS({{DriveMode::Normal, NORMAL_WHEEL_ROTS},
				  {DriveMode::TurnInPlace, TURN_IN_PLACE_WHEEL_ROTS},
				  {DriveMode::Crab, CRAB_WHEEL_ROTS}}) {}

std::pair<double, swerve_commands_t>
SwerveController::setTurnInPlaceCmdVel(double dtheta, swerve_rots_t wheel_rots) {
	if (dtheta != 0) {
		return {0, {0.0, 0.0, 0.0, 0.0}};
	}

	if (!swerveController.driveMode.second &&
		!checkWheelRotation(DriveMode::TurnInPlace, wheel_rots))
		return {0, {0.0, 0.0, 0.0, 0.0}};

	kinematics::swervewheelvel_t wheelVels =
		swerve_kinematics.robotVelToWheelVel(0, 0, dtheta);
	double lfPWM = wheelVels.lfVel / Constants::MAX_WHEEL_VEL;
	double lbPWM = wheelVels.lbVel / Constants::MAX_WHEEL_VEL;
	double rfPWM = wheelVels.rfVel / Constants::MAX_WHEEL_VEL;
	double rbPWM = wheelVels.rbVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::max(std::abs(lfPWM), std::abs(lbPWM)),
								std::max(std::abs(rfPWM), std::abs(rbPWM)));
	if (maxAbsPWM > 1) {
		lfPWM /= maxAbsPWM;
		lbPWM /= maxAbsPWM;
		rfPWM /= maxAbsPWM;
		rbPWM /= maxAbsPWM;
	}

	return {maxAbsPWM > 1 ? maxAbsPWM : 1.0, {lfPWM, rfPWM, lbPWM, rbPWM}};
}

std::pair<double, swerve_commands_t>
SwerveController::setCrabCmdVel(double dtheta, double dy, swerve_rots_t wheel_rots) {
	if (Globals::E_STOP && (dtheta != 0 || dy != 0)) {
		return {0, {0.0, 0.0, 0.0, 0.0}};
	}

	if (!swerveController.driveMode.second && !checkWheelRotation(DriveMode::Crab, wheel_rots))
		return {0, {0.0, 0.0, 0.0, 0.0}};

	kinematics::wheelvel_t wheelVels = crab_kinematics.robotVelToWheelVel(dy, dtheta);
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	return {maxAbsPWM > 1 ? maxAbsPWM : 1.0, {rPWM, rPWM, lPWM, lPWM}};
}

bool SwerveController::checkWheelRotation(DriveMode mode, swerve_rots_t wheel_rots) {
	int rots[4] = {wheel_rots.lfRot, wheel_rots.rfRot, wheel_rots.lbRot, wheel_rots.rbRot};
	for (int i = 0; i < 4; i++) {
		if (std::abs(rots[i] - WHEEL_ROTS.at(mode)[i]) < STEER_EPSILON)
			return false;
	}
	return true;
}

kinematics::SwerveDriveKinematics SwerveController::swerveKinematics() {
	return SwerveController::swerve_kinematics;
}