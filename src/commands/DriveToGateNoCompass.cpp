#include "DriveToGateNoCompass.h"

#include "../log.h"

#include "Eigen/Dense"
#include "Eigen/LU"

DriveToGateNoCompass::DriveToGateNoCompass(double driveDist, double angleKP, double vel)
	: calibrateDriveDist(driveDist), currOdom(toTransform({0,0,0})), state(State::Done),
		referenceTransform(transform_t::Zero()), calibrationPoints(),
		targetPoint({0,0,0}), angleKP(angleKP), vel(vel)
{
}
void DriveToGateNoCompass::reset(transform_t &odom, point_t &target)
{
	state = State::Start;
	referenceTransform = transform_t::Zero();
	calibrationPoints.clear();
	currOdom = odom;
	targetPoint = target;
}
command_t DriveToGateNoCompass::getOutput()
{
	if (state == State::Done || state == State::AlmostDone)
	{
		return {0, 0};
	}

	transitionStates();

	if (state == DriveForward)
	{
		return {0, vel};
	}
	else if (state == TurnToTarget)
	{
		double err = calculateHeadingErr();
		return {.thetaVel = err * angleKP, .xVel = 0};
	}

	log(LOG_WARN, "Unrecognized state: %d\n", state);
	return {0, 0};
}
bool DriveToGateNoCompass::isDone()
{
	return state == State::Done;
}
bool DriveToGateNoCompass::isAlmostDone()
{
	return state == State::AlmostDone;
}
void DriveToGateNoCompass::setDone()
{
	state = State::Done;
}
void DriveToGateNoCompass::update(const transform_t &odom, const transform_t &gps,
									const pose_t &pose, const point_t &leftPost,
									const point_t &rightPost)
{
	currOdom = odom;
	currPose = pose;
	if (state == DriveForward && gps.norm() != 0)
	{
		pose_t p = toPose(gps, 0);
		point_t point = {p.x(), p.y(), 1};
		calibrationPoints.push_back(point);
	}
	if (leftPost(2) != 0 || rightPost(2) != 0)
	{
		state = AlmostDone;
	}
}

double DriveToGateNoCompass::calculateHeadingErr()
{
	pose_t odomPose = toPose(currOdom, 0);
	pose_t targetPose = toPose(referenceTransform, odomPose(2));
	return targetPose(2) - odomPose(2);
}

Eigen::Matrix2d calculateCovMatrix(const points_t &points)
{
	// calculate center of mass
	Eigen::Vector2d com = Eigen::Vector2d::Zero();
	for (const point_t &p : points)
	{
		com += p.topRows<2>();
	}
	com /= points.size();

	// calculate the covariances
	Eigen::Vector2d variances = Eigen::Vector2d::Zero();
	double cov = 0;
	for (const point_t &p : points)
	{
		Eigen::Vector2d diff = p.topRows<2>() - com;
		Eigen::Vector2d sqrDiff = diff.array().square();
		variances += sqrDiff;
		cov += diff.x() * diff.y();
	}
	variances /= points.size();

	Eigen::Matrix2d covMat;
	covMat << variances.x(), cov, cov, variances.y();
	return covMat;
}

double calculateHeading(const points_t &points)
{
	Eigen::Matrix2d covMat = calculateCovMatrix(points);
	// covariance matrices are self-adjoint
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covMat);
	assert(solver.info() == Eigen::Success);

	// the eigenvector with the largest associated eigenvalue is the major axis of the ellipse
	// this indicates the direction the points are going
	Eigen::Vector2d majorAxis = solver.eigenvectors().rightCols<1>();
	return atan2(majorAxis.y(), majorAxis.x());
}

void DriveToGateNoCompass::transitionStates()
{
	if (state == Start)
	{
		referenceTransform = currOdom;
		state = DriveForward;
		calibrationPoints.clear();
		log(LOG_INFO, "Start->DriveForward\n");
	}
	else if (state == DriveForward)
	{
		transform_t fromReference = currOdom * referenceTransform.inverse();
		pose_t pose = toPose(fromReference, 0);
		if (pose.x() >= calibrateDriveDist)
		{
			state = State::TurnToTarget;
			double angle = calculateHeading(calibrationPoints);
			// the third element isn't 1, but it's fine since we only want displacement
			point_t toTarget = targetPoint - currPose;
			log(LOG_INFO, "Target point %f %f current pose %f %f %f\n",
					targetPoint(0), targetPoint(1),
					currPose(0), currPose(1), currPose(2));
			double targetAngle = atan2(toTarget.y(), toTarget.x());
			double angleDiff = -targetAngle + angle;
			double cosTheta = cos(angleDiff);
			double sinTheta = sin(angleDiff);
			transform_t rotMat;
			rotMat << cosTheta, -sinTheta, 0, sinTheta, cosTheta, 0, 0, 0, 1;
			// the target just the current odom rotated by the angle difference
			referenceTransform = rotMat * currOdom;
			calibrationPoints.clear();
			log(LOG_INFO, "DriveForward->TurnToTarget\n");
			log(LOG_INFO, "angle=%.0f, targetAngle=%.0f\n", angle * 180 / M_PI, targetAngle * 180/M_PI);
		}
	}
	else if (state == TurnToTarget)
	{
		log(LOG_INFO, "Heading Err: %.2f\n", calculateHeadingErr());
		if (abs(calculateHeadingErr()) <= M_PI / 16)
		{
			state = DriveForward;
			referenceTransform = currOdom;
			log(LOG_INFO, "TurnToTarget->DriveForward\n");
		}
	}
}
