#include "Autonomous.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>

#include "Globals.h"
#include "simulator/world_interface.h"

constexpr float PI = M_PI;
constexpr double KP_ANGLE = 2;
constexpr double DRIVE_SPEED = 8;


Autonomous::Autonomous(const URCLeg &_target, double controlHz)
	: target(_target),
		poseEstimator({0.8, 0.8, 0.6}, {2, 2, PI / 24}, 1.0 / controlHz),
		calibrated(false),
		calibrationPoses({}),
		landmarkFilter(),
		state(NavState::INIT)
{
}

Autonomous::Autonomous(const URCLeg &_target, double controlHz, const pose_t &startPose)
	: Autonomous(_target, controlHz)
{
	poseEstimator.reset(startPose);
	calibrated = true;
}

double dist(const pose_t &p1, const pose_t &p2, double theta_weight)
{
	pose_t diff = p1 - p2;
	// angles are modular in nature, so wrap at 2pi radians
	double thetaDiff = std::fmod(abs(diff(2)), 2 * PI);
	// change domain from [0, 2pi) to (-pi, pi]
	if (thetaDiff > PI) {
		thetaDiff -= 2 * PI;
	}
	diff(2) = thetaDiff * theta_weight;
	return diff.norm();
}

bool Autonomous::arrived(const pose_t &pose) const
{
	return dist(pose, getTargetPose(), 1.0) < 0.5;
}

double Autonomous::angleToTarget(const pose_t &gpsPose) const
{
	float dy = target.approx_GPS(1) - (float)gpsPose(1);
	float dx = target.approx_GPS(0) - (float)gpsPose(0);
	double theta = std::atan2(dy, dx);
	return theta - gpsPose(2);
}

bool calibratePeriodic(std::vector<pose_t> &poses, const pose_t &pose, pose_t &out)
{
	poses.push_back(pose);
	// 62 samples with 2m std dev and 95% confidence interval gives about +-0.5m
	if (poses.size() == 5)
	{
		pose_t sum;
		for (const pose_t &p : poses)
		{
			sum += p;
		}
		sum /= poses.size();
		out = sum;
		return true;
	}
	else
	{
		return false;
	}
}

double transformAngle(double currAngle, double targetAngle)
{
	double range = 2 * PI;
	double dist = std::fmod(targetAngle - currAngle, range);
	double absDist = abs(dist);

	int sign = dist > 0 ? 1 : (dist < 0 ? -1 : 0);

	return currAngle + ((absDist > (range / 2)) ? -sign * (range - absDist) : dist);
}

double Autonomous::getLinearVel(const pose_t &target, const pose_t &pose, double thetaErr) const {
	double speed = DRIVE_SPEED;
	if (dist(target, pose, 0) < 1.0) {
		// We should drive slower near the goal so that we don't overshoot the target
		// (given our relatively low control frequency of 10 Hz)
		speed = DRIVE_SPEED / 3;
	}
	if (abs(thetaErr) > PI / 4 || state == NavState::NEAR_TARGET_POSE) {
		// don't drive forward if pointing away
		// or if we're already very close to the target
		speed = 0;
	}
	return speed;
}

double Autonomous::getThetaVel(const pose_t &target, const pose_t &pose, double &thetaErr) const
{
	// If we're within 20cm of the target location, we want to turn the rover until
	// we reach the target orientation. Otherwise, we want to turn the rover to
	// aim it at the target location.
	double targetAngle = target(2);
	if (state == NavState::INIT) {
		double dx = target(0) - pose(0);
		double dy = target(1) - pose(1);
		targetAngle = atan2(dy, dx);
	}
	targetAngle = transformAngle(pose(2), targetAngle);
	thetaErr = targetAngle - pose(2);

	return KP_ANGLE * thetaErr;
}

void Autonomous::autonomyIter()
{
	if (!Globals::AUTONOMOUS)
	{
		return;
	}

	transform_t gps = readGPS(); // <--- has some heading information

	// If we haven't calibrated position, do so now
	if (!calibrated)
	{
		std::cout << "Calibrating..." << std::endl;
		pose_t out;
		if (calibratePeriodic(calibrationPoses, toPose(gps, 0), out))
		{
			poseEstimator.reset(out);
			calibrated = true;
			calibrationPoses.clear();
			std::cout << "Pose:\n" << out << std::endl;
		}
		else
		{
			return;
		}
	}

	// get landmark data and filter out invalid data points
	points_t landmarks = readLandmarks();
	point_t leftPostLandmark = landmarks[target.left_post_id];

	// get the latest pose estimation
	poseEstimator.correct(gps);
	pose_t pose = poseEstimator.getPose();

	if (arrived(pose))
	{
		std::cout << "arrived at gate" << std::endl;
		std::cout << "x: " << pose(0) << " y: " << pose(1) << " theta: " << pose(2)
					<< std::endl;
		landmarkFilter.reset(); // clear the cached data points
		setCmdVel(0, 0);
	}
	else
	{
		pose_t driveTarget = getTargetPose();

		// if we have some existing data or new data, set the target using the landmark
		// data
		bool landmarkVisible = leftPostLandmark[2] != 0;
		if (landmarkFilter.getSize() > 0 || landmarkVisible)
		{
			// TODO shift the target location and orientation to align
			// with the gate and/or avoid crashing into the post.
			if (!landmarkVisible)
			{
				// we have no new data, so use the data already in the filter
				driveTarget.topRows(2) = landmarkFilter.get().topRows(2);
			}
			else
			{
				// transform and add the new data to the filter
				transform_t invTransform = toTransform(pose).inverse();
				point_t landmarkMapSpace = invTransform * leftPostLandmark;
				// the filtering is done on the target in map space to reduce any phase lag
				// caused by filtering
				driveTarget.topRows(2) = landmarkFilter.get(landmarkMapSpace).topRows(2);
			}
		}

		double d = dist(driveTarget, pose, 0);
		// There's an overlap where either state might apply, to prevent rapidly switching
		// back and forth between these two states.
		if (d < 0.2) {
			state = NavState::NEAR_TARGET_POSE;
		}
		if (d > 0.5) {
			state = NavState::INIT;
		}
		double thetaErr;
		double thetaVel = getThetaVel(driveTarget, pose, thetaErr);
		double driveSpeed = getLinearVel(driveTarget, pose, thetaErr);

		if (!Globals::E_STOP)
		{
			setCmdVel(thetaVel, driveSpeed);
			poseEstimator.predict(thetaVel, driveSpeed);

			std::cout << "ThetaVel: " << thetaVel << " DriveVel: " << driveSpeed
						<< " thetaErr: " << thetaErr << " targetX: " << driveTarget(0)
						<< " targetY: " << driveTarget(1) << std::endl;
		}
	}
}

double Autonomous::pathDirection(const points_t &lidar, const pose_t &gpsPose)
{
	double dtheta;
	dtheta = angleToTarget(gpsPose);
	return dtheta;
}

pose_t Autonomous::getTargetPose() const
{
	pose_t ret { target.approx_GPS(0), target.approx_GPS(1), 0.0 };
	return ret;
}
