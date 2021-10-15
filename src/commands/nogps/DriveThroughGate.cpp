#include "DriveThroughGate.h"

#include "../../log.h"
#include "Eigen/LU"

#include <cmath>

constexpr double ALIGN_DIST = 2;

DriveThroughGate::DriveThroughGate(double thetaKP, double slowVel, double fastVel)
	: startOdom(toTransform({0, 0, 0})), driveTarget(pose_t::Zero()), thetaKP(thetaKP),
	  slowVel(slowVel), fastVel(fastVel), state(State::Done) {
}

void DriveThroughGate::reset(transform_t& odom) {
	startOdom = odom;
	state = State::Start;
	driveTarget = pose_t::Zero();
}

bool DriveThroughGate::isDone() {
	return state == State::Done;
}

double headingErr(const transform_t& trf, const pose_t& target) {
	return target(2) - toPose(trf, target(2))(2);
}

double posErr(const transform_t& trf, const pose_t& target) {
	point_t targetPoint = {target.x(), target.y(), 1};
	return (trf * targetPoint).topRows<2>().norm();
}

pose_t calculateStartTarget(const transform_t& trf, const point_t& leftPost,
							const point_t& rightPost) {
	point_t post = leftPost(2) == 0 ? rightPost : leftPost;

	double angle = atan2(post.y(), post.x());
	pose_t target = toPose(trf, 0);
	target(2) += angle;
	return target;
}

pose_t calculateAlignTargetPose(const transform_t& trf, const point_t& leftPost,
								const point_t& rightPost) {
	transform_t trfInv = trf.inverse();

	point_t center = (leftPost + rightPost) / 2;
	center(2) = 1;

	Eigen::Vector2d centerToRobot = (-center).topRows<2>();
	point_t gateDiff = (rightPost - leftPost);
	gateDiff(2) = 1;

	// get the normalized vector normal to the gate
	transform_t rotate;
	rotate << 0, -1, 0, 1, 0, 0, 0, 0, 1;
	Eigen::Vector2d gateNormal = (rotate * gateDiff).topRows<2>();
	gateNormal.normalize();

	// now project centerToRobot onto gateNormal and scale to get the offset
	double dot = centerToRobot.dot(gateNormal);
	// handle the case where the dot product is zero
	if (fabs(dot) <= 1e-9) {
		dot = 1;
	}
	Eigen::Vector2d offset = ALIGN_DIST * (dot * gateNormal).normalized();
	point_t target = {center.x() + offset.x(), center.y() + offset.y(), 1};
	// convert to "global space"
	point_t targetGlobal = trfInv * target;

	// calculate the target angle
	point_t centerGlobal = trfInv * center;
	Eigen::Vector2d targetToCenter = (centerGlobal - targetGlobal).topRows<2>();
	double angle = atan2(targetToCenter.y(), targetToCenter.x());

	return {targetGlobal.x(), targetGlobal.y(), angle};
}

command_t DriveThroughGate::getOutput() {
	transitionStates();

	transform_t trf = getTrfFromStart();
	transform_t trfInv = trf.inverse();
	pose_t pose = toPose(trf, 0);

	if (state == OneVisibleDrive) {
		return getCommandToTarget(trf, driveTarget);
	} else if (state == OneVisibleTurn) {
		double thetaVel = thetaKP * headingErr(trf, driveTarget);
		return {.thetaVel = thetaVel, .xVel = 0};
	} else if (state == BothVisible) {
		// recalculate the driveTarget, if possible
		if (leftPost(2) != 0 && rightPost(2) != 0) {
			driveTarget = calculateAlignTargetPose(trf, leftPost, rightPost);
		}
		return getCommandToTarget(trf, driveTarget);
	} else if (state == Align) {
		// recalculate the driveTarget, if possible
		if (leftPost(2) != 0 && rightPost(2) != 0) {
			driveTarget = calculateAlignTargetPose(trf, leftPost, rightPost);
		}
		double thetaVel = thetaKP * headingErr(trf, driveTarget);
		return {.thetaVel = thetaVel, .xVel = 0};
	} else if (state == DriveThrough) {
		command_t cmd = getCommandToTarget(trf, driveTarget);
		cmd.xVel = slowVel; // always go slow
		return cmd;
	} else {
		return {0, 0};
	}
}

void DriveThroughGate::update(const transform_t& odom, const point_t& left,
							  const point_t& right) {
	currOdom = odom;
	leftPost = left;
	rightPost = right;
}

pose_t calculateNextSearchPose(const point_t& postLocal, const transform_t& trf) {
	transform_t trfInv = trf.inverse();

	// create rotation matrix 90 deg CCW
	transform_t rotate;
	rotate << 0, -1, 0, 1, 0, 0, 0, 0, 1;

	point_t offset = rotate * postLocal;
	point_t targetLocal = {postLocal.x() + offset.x(), postLocal.y() + offset.y(), 1};

	point_t target = trfInv * targetLocal;
	point_t post = trfInv * postLocal;
	Eigen::Vector2d targetToPost = (post - target).topRows<2>();
	double angle = atan2(targetToPost.y(), targetToPost.x());
	target(2) = angle;

	return target;
}

void DriveThroughGate::transitionStates() {
	transform_t trf = getTrfFromStart();
	pose_t pose = toPose(trf, 0);
	switch (state) {
		case Start:
			driveTarget = calculateStartTarget(trf, leftPost, rightPost);
			state = OneVisibleTurn;
			log(LOG_INFO, "Start->OneVisibleTurn\n");
			break;

		case OneVisibleTurn:
			if (leftPost(2) == 1 && rightPost(2) == 1) {
				driveTarget = calculateAlignTargetPose(trf, leftPost, rightPost);
				log(LOG_INFO, "OneVisibleTurn->BothVisible\n");
				state = BothVisible;
			} else if (abs(headingErr(trf, driveTarget)) <= M_PI / 9) {
				point_t postLocal = leftPost(2) != 0 ? leftPost : rightPost;
				driveTarget = calculateNextSearchPose(postLocal, trf);
				state = OneVisibleDrive;
				log(LOG_INFO, "OneVisibleTurn->OneVisibleDrive\n");
			}
			break;

		case OneVisibleDrive:
			if (leftPost(2) == 1 && rightPost(2) == 1) {
				driveTarget = calculateAlignTargetPose(trf, leftPost, rightPost);
				state = BothVisible;
				log(LOG_INFO, "OneVisibleDrive->BothVisible\n");
			} else if (posErr(trf, driveTarget) <= 0.75) {
				state = OneVisibleTurn;
				log(LOG_INFO, "OneVisibleDrive->OneVisibleTurn\n");
			}
			break;

		case BothVisible:
			if (posErr(trf, driveTarget) <= 0.5) {
				state = Align;
				log(LOG_INFO, "BothVisible->Align\n");
			}
			break;

		case Align:
			if (abs(headingErr(trf, driveTarget)) <= M_PI / 9) {
				// move the drive target twice the align distance in the current direction
				double angle = driveTarget(2);
				double cosTheta = cos(angle);
				double sinTheta = sin(angle);
				double dist = 2 * ALIGN_DIST;
				driveTarget = {driveTarget.x() + dist * cosTheta,
							   driveTarget.y() + dist * sinTheta, angle};
				state = DriveThrough;
				log(LOG_INFO, "Align->DriveThrough\n");
			}
			break;

		case DriveThrough:
			if (posErr(trf, driveTarget) <= 0.5) {
				state = Done;
				log(LOG_INFO, "DriveThrough->Done\n");
			}
			break;

		default:
		case Done:
			break;
	}
}

transform_t DriveThroughGate::getTrfFromStart() {
	// currOdom = A * startOdom
	// A = currOdom * startOdom^-1
	return currOdom * startOdom.inverse();
}

command_t DriveThroughGate::getCommandToTarget(const transform_t& trf, const point_t& target) {
	point_t targetPoint = {target.x(), target.y(), 1};
	point_t targetLocal = trf * targetPoint;
	if (targetLocal.isZero()) {
		return {0, 0};
	}

	double dist = targetLocal.topRows<2>().norm();
	double vel = dist <= 1.5 ? slowVel : fastVel;
	double angleErr = atan2(targetLocal.y(), targetLocal.x());
	if (angleErr >= M_PI / 6) {
		vel = 0;
	}
	double thetaVel = thetaKP * angleErr;
	return {.thetaVel = thetaVel, .xVel = vel};
}
