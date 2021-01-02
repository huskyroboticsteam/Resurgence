#include "Autonomous.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>

#include "Globals.h"
#include "simulator/world_interface.h"
#include "simulator/constants.h"

constexpr float PI = M_PI;
constexpr double KP_ANGLE = 2;
constexpr double DRIVE_SPEED = 8;
constexpr double SEARCH_THETA_INCREMENT = PI / 4;
const Eigen::Matrix2d searchPatternTransform =
	(Eigen::Matrix2d() << cos(SEARCH_THETA_INCREMENT), -sin(SEARCH_THETA_INCREMENT),
	 					  sin(SEARCH_THETA_INCREMENT), cos(SEARCH_THETA_INCREMENT)).finished();
const Eigen::Vector3d gpsStdDev = {2, 2, PI / 24};
constexpr int numSamples = 1;

const transform_t VIZ_BASE_TF = toTransform({NavSim::DEFAULT_WINDOW_CENTER_X,NavSim::DEFAULT_WINDOW_CENTER_Y,M_PI/2});

Autonomous::Autonomous(const URCLeg &_target, double controlHz)
	: viz_window("Planning visualization"),
		target(_target),
		driveTarget({target.approx_GPS(0), target.approx_GPS(1), 0.0}),
		poseEstimator({1.5, 1.5}, gpsStdDev, Constants::WHEEL_BASE, 1.0 / controlHz),
		calibrated(false),
		calibrationPoses({}),
		landmarkFilter(),
		state(NavState::INIT),
		clock_counter(0),
		plan(0,2),
		plan_base({0,0,0}),
		plan_idx(0)
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

void Autonomous::drawPose(pose_t &pose, pose_t &current_pose, sf::Color c)
{
	// Both poses are given in the same frame. (usually GPS frame)
	// `current_pose` is the current location of the robot, `pose` is the pose we wish to draw
	transform_t inv_curr = toTransform(current_pose).inverse();
	viz_window.drawRobot(toTransform(pose) * inv_curr * VIZ_BASE_TF, c);
}

bool Autonomous::arrived(const pose_t &pose) const
{
	return landmarkFilter.getSize() > 0 && dist(pose, driveTarget, 1.0) < 0.5;
}

double Autonomous::angleToTarget(const pose_t &gpsPose) const
{
	float dy = driveTarget(1) - (float)gpsPose(1);
	float dx = driveTarget(0) - (float)gpsPose(0);
	double theta = atan2(dy, dx);
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
	if (state == NavState::INIT || state == NavState::SEARCH_PATTERN)
	{
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
		// if we have some existing data or new data, set the target using the landmark
		// data
		bool landmarkVisible = leftPostLandmark(2) != 0;
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
			if (state == NavState::SEARCH_PATTERN)
			{
				// Currently in a search pattern and landmark has been seen, can exit search
				state = NavState::INIT;
			}
		}

		const points_t lidar_scan = readLidarScan();
		bool should_replan = ((clock_counter++) % 20 == 0); // TODO make this configurable
		if (should_replan) {
			plan_base = pose;
			plan_idx = 0;
			point_t point_t_goal;
			point_t_goal.topRows(2) = driveTarget.topRows(2);
			point_t_goal(2) = 1.0;
			point_t_goal = toTransform(pose) * point_t_goal;
			double goal_radius = 2.0;
			plan = getPlan(lidar_scan, point_t_goal, goal_radius);
		}

		while (viz_window.pollWindowEvent() != -1) {}
		viz_window.drawPoints(transformReadings(lidar_scan, VIZ_BASE_TF), sf::Color::Red, 3);
		drawPose(pose, pose, sf::Color::Black);

		int plan_size = plan.rows();
		if (plan_size == 0 && dist(driveTarget, pose, 1.0) < 2.0) {
			// We're probably within planning resolution of the goal,
			// so using the goal as the drive target makes sense.
		} else {
			// Roll out the plan until we find a target pose a certain distance in front
			// of the robot. (This code also handles visualizing the plan.)
			pose_t plan_pose = plan_base;
			drawPose(plan_pose, pose, sf::Color::Red);
			bool found_target = false;
			for (int i = 0; i < plan_size; i++) {
				action_t action = plan.row(i);
				plan_pose(2) += action(0);
				plan_pose(0) += action(1) * cos(plan_pose(2));
				plan_pose(1) += action(1) * sin(plan_pose(2));
				if (i >= plan_idx && !found_target && dist(plan_pose, pose, 1.0) > 5.0) {
					found_target = true;
					plan_idx = i;
					driveTarget = plan_pose;
					drawPose(plan_pose, pose, sf::Color::Blue);
				} else {
					drawPose(plan_pose, pose, sf::Color::Red);
				}
			}
		}
		viz_window.display();

		double d = dist(driveTarget, pose, 0);
		if (d < 0.2 && state == NavState::SEARCH_PATTERN) {
			// Current search point has been reached, set the drive target to the
			// next point in the search pattern
			driveTarget -= target.approx_GPS;
			double radius = hypot(driveTarget(0), driveTarget(1));
			double scale = (radius + SEARCH_THETA_INCREMENT) / radius;
			driveTarget.topRows(2) = searchPatternTransform * driveTarget.topRows(2) * scale;
			driveTarget += target.approx_GPS;
			driveTarget(2) += SEARCH_THETA_INCREMENT;
			if (driveTarget(2) > PI)
			{
				driveTarget(2) -= 2 * PI;
			}
		}
		if (d < 0.2 && landmarkFilter.getSize() == 0 && !landmarkVisible)
		{
			// Close to GPS target but no landmark in sight, should use search pattern
			state = NavState::SEARCH_PATTERN;
			driveTarget = {target.approx_GPS(0) - PI, target.approx_GPS(1), -PI / 2};
		}
		// There's an overlap where either state might apply, to prevent rapidly switching
		// back and forth between these two states.
		// We also don't want to switch into one of these two states if we're currently in a
		// search pattern.
		if (d < 0.2 && state == NavState::INIT)
		{
			state = NavState::NEAR_TARGET_POSE;
		}
		if (d > 0.5 && state == NavState::NEAR_TARGET_POSE)
		{
			state = NavState::INIT;
		}
		double thetaErr;
		double thetaVel = getThetaVel(driveTarget, pose, thetaErr);
		double driveSpeed = getLinearVel(driveTarget, pose, thetaErr);

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
	dtheta = angleToTarget(gpsPose);
	return dtheta;
}

pose_t Autonomous::getTargetPose() const
{
	return driveTarget;
}
