#include "DriveToGateNoCompass.h"
#include <iostream>

#include "../log.h"

#include "Eigen/Dense"
#include "Eigen/LU"

DriveToGateNoCompass::DriveToGateNoCompass(double driveDist, double angleKP, double vel, double initial_heading)
	: calibrateDriveDist(driveDist), currOdom(toTransform({0,0,0})), state(State::Done),
		calibrationPoints(),
		targetPoint({0,0,0}), angleKP(angleKP), vel(vel), currPose({0,0,0}),
		checkpointHeading(initial_heading)
{
}
void DriveToGateNoCompass::reset(transform_t &odom, point_t &target)
{
	state = State::Start;
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
		calibrationPoints.clear();
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
void DriveToGateNoCompass::setDone(const transform_t &odom)
{
	currOdom = odom;
	calibrationPoints.clear();
	checkpoint();
	log(LOG_INFO, "Estimated heading on completion: %.2f\n", getCurrentHeading());
	state = State::Done;
}
void DriveToGateNoCompass::update(const transform_t &odom, const transform_t &gps,
									const pose_t &pose, const point_t &leftPost,
									const point_t &rightPost)
{
	currOdom = odom;
	currPose(0) = pose(0);
	currPose(1) = pose(1);
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

double DriveToGateNoCompass::headingChangeSinceCheckpoint()
{
	pose_t odomPose = toPose(currOdom, 0);
	pose_t checkpointPose = toPose(checkpointOdom, 0);
	return odomPose(2) - checkpointPose(2);
}

double DriveToGateNoCompass::distFromCheckpoint()
{
	pose_t odomPose = toPose(currOdom, 0);
	pose_t checkpointPose = toPose(checkpointOdom, 0);
	pose_t diff = odomPose - checkpointPose;
	diff(2) = 0;
	return diff.norm();
}

double DriveToGateNoCompass::getCurrentHeading()
{
	double delta = headingChangeSinceCheckpoint();
	return checkpointHeading + delta;
}

double DriveToGateNoCompass::calculateHeadingErr()
{
	double diff = getTargetHeading() - getCurrentHeading();
	return nearestHeadingBranch(diff, 0.0);
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

	Eigen::Matrix2d covMat;
	// calculate the covariances
	//Eigen::Vector2d variances = Eigen::Vector2d::Zero();
	//double cov = 0;
	for (const point_t &p : points)
	{
		Eigen::Vector2d diff = p.topRows<2>() - com;
		covMat += diff * diff.transpose();
		//Eigen::Vector2d sqrDiff = diff.array().square();
		//variances += sqrDiff;
		//cov += diff.x() * diff.y();
	}
	//variances /= points.size();

	//covMat << variances.x(), cov, cov, variances.y();
	return covMat;
}

double calculateHeading(const points_t &points, double odom_heading)
{
	size_t n_pts = points.size();
	double rise = points[n_pts-1](1) - points[0](1);
	double run = points[n_pts-1](0) - points[0](0);
	double gps_heading = atan2(rise, run);
	/*
	Eigen::Matrix2d covMat = calculateCovMatrix(points);
	// covariance matrices are self-adjoint
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covMat);
	assert(solver.info() == Eigen::Success);

	// the eigenvector with the largest associated eigenvalue is the major axis of the ellipse
	// this indicates the direction the points are going
	Eigen::Vector2d majorAxis = solver.eigenvectors().rightCols<1>();
	double eigvect_heading = atan2(majorAxis.y(), majorAxis.x());
	*/
	log(LOG_INFO, "calculateHeading %f %f\n", gps_heading, odom_heading);
	gps_heading = nearestHeadingBranch(gps_heading, odom_heading);
	// We only know this eigenvector up to sign
	if (gps_heading - odom_heading > M_PI/2) gps_heading += M_PI;
	return nearestHeadingBranch(gps_heading, odom_heading);
}

void DriveToGateNoCompass::checkpoint()
{
	size_t n_pts = calibrationPoints.size();
	double odom_heading = getCurrentHeading();
	if (calibrationPoints.size() >= 3)
	{
		log(LOG_INFO, "Using GPS-based heading calculation from %d points\n", n_pts);
		for (point_t &p : calibrationPoints)
		{
			std::cout << p << std::endl;
		}
		checkpointHeading = calculateHeading(calibrationPoints, odom_heading); // gps-based
	}
	else
	{
		log(LOG_INFO, "Using odom heading calculation\n");
		checkpointHeading = odom_heading; // odom-based
	}
	checkpointOdom = currOdom;
}

double DriveToGateNoCompass::getTargetHeading()
{
	// the third element isn't 1, but it's fine since we only want displacement
	point_t toTarget = targetPoint - currPose;
	return atan2(toTarget.y(), toTarget.x());
}

void DriveToGateNoCompass::transitionStates()
{
	if (state == Start)
	{
		state = TurnToTarget;
		checkpoint();
		calibrationPoints.clear();
		log(LOG_INFO, "Start->TurnToTarget\n");
	}
	else if (state == DriveForward)
	{
		if (distFromCheckpoint() >= calibrateDriveDist)
		{
			checkpoint();
			state = State::TurnToTarget;
			log(LOG_INFO, "DriveForward->TurnToTarget\n");
		}
	}
	else if (state == TurnToTarget)
	{
		double err = calculateHeadingErr();
		log(LOG_INFO, "Heading Err: %.2f\n", err);
		if (abs(err) <= M_PI / 16)
		{
			state = DriveForward;
			checkpoint();
			log(LOG_INFO, "TurnToTarget->DriveForward\n");
		}
	}
	log(LOG_INFO, "Estimated heading: %.2f Target: %.2f\n",
			getCurrentHeading(), getTargetHeading());
}
