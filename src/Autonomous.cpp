#include "Autonomous.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>

#include "Globals.h"
#include "simulator/world_interface.h"

constexpr float PI = M_PI;
constexpr double KP_ANGLE = 2;
constexpr double DRIVE_SPEED = 8;
const Eigen::Vector3d gpsStdDev = {2, 2, PI / 24};
constexpr int numSamples = 1;

Autonomous::Autonomous(const URCLeg &_target, double controlHz)
	: target(_target),
		poseEstimator({1.2, 1.2}, gpsStdDev, 1.0 / controlHz),
		state(0),
		targetHeading(-1),
		forwardCount(-1),
		rightTurn(false),
		calibrated(false),
		calibrationPoses({}),
		landmarkFilter()
{
}

Autonomous::Autonomous(const URCLeg &_target, double controlHz, const pose_t &startPose)
	: Autonomous(_target, controlHz)
{
	poseEstimator.reset(startPose);
	calibrated = true;
}

bool Autonomous::arrived(const pose_t &pose) const
{
	double currX = pose[0];
	double currY = pose[1];
	return util::almostEqual(currX, (double)target.approx_GPS(0), 0.5) &&
		   util::almostEqual(currY, (double)target.approx_GPS(1), 0.5);
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
	if (poses.size() == numSamples)
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

double getThetaVel(const point_t &target, const pose_t &pose, double &thetaErr)
{
	double dx = target(0) - pose(0);
	double dy = target(1) - pose(1);
	double targetAngle = atan2(dy, dx);
	targetAngle = transformAngle(pose(2), targetAngle);
	thetaErr = targetAngle - pose(2);

	return KP_ANGLE * thetaErr;
}

double dist(const point_t &p1, const point_t &p2)
{
	// TODO do we want to weight the theta difference differently?
	return (p1 - p2).norm();
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
		pose_t out;
		if (calibratePeriodic(calibrationPoses, toPose(gps, 0), out))
		{
			// the standard error of the calculated mean is the std dev of the mean
			poseEstimator.reset(out, gpsStdDev / sqrt((double)numSamples));
			calibrated = true;
			calibrationPoses.clear();
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
		double thetaErr;
		double thetaVel = getThetaVel(driveTarget, pose, thetaErr);
		double driveSpeed =
			abs(thetaErr) < PI / 4 ? DRIVE_SPEED : 0; // don't drive forward if pointing away

		if (dist(pose, driveTarget) < 4)
		{
			driveSpeed /= 3.0;
		}

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

pose_t Autonomous::getTargetPose()
{
	pose_t ret{target.approx_GPS(0), target.approx_GPS(1), 0.0};
	return ret;
}
