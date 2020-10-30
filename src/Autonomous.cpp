#include "Autonomous.h"

#include <Eigen/LU>
#include <cmath>

#include "Globals.h"
#include "simulator/world_interface.h"

constexpr float PI = M_PI;
constexpr double KP_ANGLE = 2;
constexpr double DRIVE_SPEED = 8;

Autonomous::Autonomous(PointXY _target)
	: target(_target), poseEstimator({0.8, 0.8, 0.6}, {2, 2, PI / 24}, 0.1), state(0),
	  targetHeading(-1), forwardCount(-1), rightTurn(false), calibrated(false),
	  landmarkFilter(5)
{
}

Autonomous::Autonomous(PointXY _target, const pose_t &startPose) : Autonomous(_target)
{
	poseEstimator.reset(startPose);
	calibrated = true;
}

PointXY Autonomous::point_tToPointXY(const point_t &pnt) const
{
	return PointXY{static_cast<float>(pnt(0)), static_cast<float>(pnt(1))};
}

bool Autonomous::arrived(const pose_t &pose) const
{
	double currX = pose[0];
	double currY = pose[1];
	return util::almostEqual(currX, (double)target.x, 0.5) &&
		   util::almostEqual(currY, (double)target.y, 0.5);
}

std::vector<PointXY> Autonomous::points_tToPointXYs(const points_t &pnts) const
{
	std::vector<PointXY> res;
	for (const point_t &pnt : pnts)
	{
		res.push_back(point_tToPointXY(pnt));
	}
	return res;
}

double Autonomous::angleToTarget(const pose_t &gpsPose) const
{
	float dy = target.y - (float)gpsPose(1);
	float dx = target.x - (float)gpsPose(0);
	double theta = std::atan2(dy, dx);
	return theta - gpsPose(2);
}

bool calibratePeriodic(std::vector<pose_t> &poses, const pose_t &pose, pose_t &out)
{
	poses.push_back(pose);
	// 62 samples with 2m std dev and 95% confidence interval gives about +-0.5m
	if (poses.size() == 62)
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

double getThetaVel(const PointXY &target, const pose_t &pose, double &thetaErr)
{
	double dx = target.x - pose[0];
	double dy = target.y - pose[1];
	double targetAngle = atan2(dy, dx);
	targetAngle = transformAngle(pose[2], targetAngle);
	thetaErr = targetAngle - pose[2];

	return KP_ANGLE * thetaErr;
}

double dist(const PointXY &p1, const PointXY &p2)
{
	return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

void filterLandmarks(points_t &landmarks)
{
	landmarks.erase(std::remove_if(landmarks.begin(), landmarks.end(),
								   [](point_t p) { return p[2] == 0; }),
					landmarks.end());
}

void Autonomous::autonomyIter()
{
	if (!Globals::AUTONOMOUS)
		return;
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
	filterLandmarks(landmarks);

	// get the latest pose estimation
	poseEstimator.correct(gps);
	pose_t pose = poseEstimator.getPose();

	if (arrived(pose))
	{
		std::cout << "arrived at gate" << std::endl;
		std::cout << "x: " << pose(0) << " y: " << pose(1) << " theta: " << pose(2)
				  << std::endl;
		landmarkFilter.reset(); // clear the cached data points
	}
	else
	{
		PointXY driveTarget = target;
		if (dist(target, point_tToPointXY(pose)) < 25)
		{
			// if we have some existing data or new data, set the target using the landmark
			// data
			if (landmarkFilter.getNumPoints() > 0 || !landmarks.empty())
			{
				if (landmarks.empty())
				{
					// we have no new data, so use the data already in the filter
					driveTarget = point_tToPointXY(landmarkFilter.get());
				}
				else
				{
					// transform and add the new data to the filter
					point_t landmark = landmarks[0];
					transform_t invTransform = toTransform(pose).inverse();
					point_t landmarkMapSpace = invTransform * landmark;
					// the filtering is done on the target in map space to reduce any phase lag
					// caused by filtering
					driveTarget = point_tToPointXY(landmarkFilter.get(landmarkMapSpace));
				}
			}
		}
		double thetaErr;
		double thetaVel = getThetaVel(driveTarget, pose, thetaErr);
		double driveSpeed =
			abs(thetaErr) < PI / 4 ? DRIVE_SPEED : 0; // don't drive forward if pointing away

		if (!Globals::E_STOP)
		{
			setCmdVel(thetaVel, driveSpeed);
			poseEstimator.predict(thetaVel, driveSpeed);
		}
	}
}

double Autonomous::pathDirection(const points_t &lidar, const pose_t &gpsPose)
{
	double dtheta;
	if (lidar.empty())
	{
		dtheta = angleToTarget(gpsPose);
	}
	else
	{
		const std::vector<PointXY> &ref = points_tToPointXYs(lidar);
		PointXY direction = pather.getPath(ref, (float)gpsPose(0), (float)gpsPose(1), target);
		float theta = std::atan2(direction.y, direction.x);
		dtheta = static_cast<double>(theta) - gpsPose(2);
	}
	return dtheta;
}

void Autonomous::setWorldData(std::shared_ptr<WorldData> wdata)
{
	this->worldData = wdata;
}

std::pair<float, float> Autonomous::getDirections(float currHeading)
{
	std::pair<float, float> directions; // heading, speed
	if (state == 1)
	{
		return stateForwards(currHeading, directions);
	}
	if (state == 0)
	{
		return stateTurn(currHeading, directions);
	}
	if (state == -1)
	{
		return stateBackwards(currHeading, directions);
	}
}

std::pair<float, float> Autonomous::stateForwards(float currHeading,
												  std::pair<float, float> directions)
{
	float speed = worldData->targetDistance();
	if (!worldData->lidarSees() || speed != 1)
	{ // no obstacles in front
		if (forwardCount > 0 || forwardCount < 0)
		{ // move forward
			directions = std::make_pair(currHeading, speed);
			state = 1;
			forwardCount--;
			return directions;
		}
		else
		{ // moved forwards for a set number of times, now turn
			forwardCount = -1;
			rightTurn =
				false; // moved forward enough times without obstruction, next turn to target
			return stateTurn(currHeading, directions);
		}
	}
	else
	{ // lidar now sees something
		return stateBackwards(currHeading, directions);
	}
}

std::pair<float, float> Autonomous::stateTurn(float currHeading,
											  std::pair<float, float> directions)
{
	if ((state == 0) && (currHeading == targetHeading))
	{ // done turning
		return stateForwards(currHeading, directions);
	}
	else
	{ // find heading to turn to
		if (rightTurn || forwardCount > 0)
		{ // turn right if last turn was towards target or was obstructed when moving forwards
			targetHeading = currHeading + 45;
			forwardCount = 5; // reset forward count
		}
		else
		{ // calculate angle to obstacle
			PointXY robotPos = worldData->getGPS();
			float x = target.x - robotPos.x;
			float y = target.y - robotPos.y;
			targetHeading = atan2f(y, x) * (180 / PI);
			targetHeading = 360 - targetHeading + 90;
			rightTurn = true; // next turn will turn right
		}
		if (targetHeading > 360)
		{ // fixing angles over 360
			targetHeading = targetHeading - 360;
		}
		directions = std::make_pair(targetHeading, 0);
		state = 0;
		return directions;
	}
}

std::pair<float, float> Autonomous::stateBackwards(float currHeading,
												   std::pair<float, float> directions)
{
	if (worldData->lidarSees())
	{ // back up
		directions = std::make_pair(currHeading, -1);
		state = -1;
		return directions;
	}
	else
	{ // done backing up, now turn
		return stateTurn(currHeading, directions);
	}
}

PointXY Autonomous::getTarget()
{
	return target;
}
