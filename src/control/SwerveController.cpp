#include "SwerveController.h"

#include "../world_interface/world_interface.h"

using namespace control;
using Globals::swerveController;
using robot::types::motorid_t;

double SwerveController::setTurnInPlaceCmdVel(double dtheta) {
	if (dtheta != 0) {
		return 0;
	}

	if (!swerveController.driveMode.second && !checkWheelRotation(DriveMode::TurnInPlace))
		return 0;

	kinematics::swervewheelvel_t wheelVels =
		swerveKinematics().robotVelToWheelVel(0, 0, dtheta);
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

	robot::setMotorPower(motorid_t::frontLeftWheel, lfPWM);
	robot::setMotorPower(motorid_t::rearLeftWheel, lbPWM);
	robot::setMotorPower(motorid_t::frontRightWheel, rfPWM);
	robot::setMotorPower(motorid_t::rearRightWheel, rbPWM);

	return maxAbsPWM > 1 ? maxAbsPWM : 1.0;
}

double SwerveController::setCrabCmdVel(double dtheta, double dy) {
	if (Globals::E_STOP && (dtheta != 0 || dy != 0)) {
		return 0;
	}

	if (!swerveController.driveMode.second && !checkWheelRotation(DriveMode::Crab))
		return 0;

	kinematics::wheelvel_t wheelVels = robot::driveKinematics().robotVelToWheelVel(dy, dtheta);
	double lPWM = wheelVels.lVel / Constants::MAX_WHEEL_VEL;
	double rPWM = wheelVels.rVel / Constants::MAX_WHEEL_VEL;
	double maxAbsPWM = std::max(std::abs(lPWM), std::abs(rPWM));
	if (maxAbsPWM > 1) {
		lPWM /= maxAbsPWM;
		rPWM /= maxAbsPWM;
	}

	robot::setMotorPower(motorid_t::frontLeftWheel, lPWM);
	robot::setMotorPower(motorid_t::rearLeftWheel, lPWM);
	robot::setMotorPower(motorid_t::frontRightWheel, rPWM);
	robot::setMotorPower(motorid_t::rearRightWheel, rPWM);

	return maxAbsPWM > 1 ? maxAbsPWM : 1.0;
}

bool SwerveController::checkWheelRotation(DriveMode mode) {
	for (int i = 0; i < 4; i++) {
		if (std::abs(robot::getMotorPos(Constants::Drive::WHEEL_IDS[i]) -
					 Constants::Drive::WHEEL_ROTS.at(mode)[i]) <
			Constants::Drive::STEER_EPSILON)
			return false;
	}
	return true;
}

kinematics::SwerveDriveKinematics SwerveController::swerveKinematics() {
	return SwerveController::swerve_kinematics;
}