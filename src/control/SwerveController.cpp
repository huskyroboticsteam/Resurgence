#include "SwerveController.h"

#include "../Constants.h"
#include "../Globals.h"

using namespace control;
using Globals::swerveController;
using robot::types::motorid_t;

std::pair<double, std::vector<double>>
SwerveController::setTurnInPlaceCmdVel(double dtheta, std::vector<int> wheel_rots) {
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

std::pair<double, std::vector<double>>
SwerveController::setCrabCmdVel(double dtheta, double dy, std::vector<int> wheel_rots) {
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

bool SwerveController::checkWheelRotation(DriveMode mode, std::vector<int> wheel_rots) {
	for (int i = 0; i < 4; i++) {
		if (std::abs(wheel_rots[i] - Constants::Drive::WHEEL_ROTS.at(mode)[i]) <
			Constants::Drive::STEER_EPSILON)
			return false;
	}
	return true;
}

kinematics::SwerveDriveKinematics SwerveController::swerveKinematics() {
	return SwerveController::swerve_kinematics;
}