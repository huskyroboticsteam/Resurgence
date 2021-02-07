#include "Autonomous.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>
#include <csignal>

#include "Globals.h"
#include "simulator/world_interface.h"
#include "simulator/constants.h"

constexpr float PI = M_PI;
constexpr double KP_ANGLE = 2;
constexpr double DRIVE_SPEED = 3;
const Eigen::Vector3d gpsStdDev = {2, 2, PI / 24};
constexpr int numSamples = 1;

constexpr double FOLLOW_DIST = 5.0;
constexpr double PLAN_COLLISION_STOP_DIST = 2.0;
constexpr double INFINITE_COST = 1e10;
constexpr int REPLAN_PERIOD = 20;

const transform_t VIZ_BASE_TF = toTransform({NavSim::DEFAULT_WINDOW_CENTER_X,NavSim::DEFAULT_WINDOW_CENTER_Y,M_PI/2});

void closeSim(int signum)
{
	rclcpp::shutdown();
	raise(SIGTERM);
}

Autonomous::Autonomous(const URCLeg &_target, double controlHz)
	: target(_target),
		search_target({target.approx_GPS(0) - PI, target.approx_GPS(1), -PI / 2}),
		poseEstimator({1.5, 1.5}, gpsStdDev, Constants::WHEEL_BASE, 1.0 / controlHz),
		calibrated(false),
		calibrationPoses({}),
		landmarkFilter(),
		state(NavState::INIT),
		time_since_plan(0),
		plan(0,2),
		plan_cost(INFINITE_COST),
		plan_base({0,0,0}),
		plan_idx(0),
		search_theta_increment(PI / 4),
		already_arrived(false)
{
	// Ctrl+C doesn't stop the simulation without this line
	signal(SIGINT, closeSim);

	// Initialize ROS without any commandline arguments
	rclcpp::init(0, nullptr);
	node = std::make_shared<rclcpp::Node>("autonomous");
	plan_pub = node->create_publisher<geometry_msgs::msg::Point>("plan_viz", 100);
	curr_pose_pub = node->create_publisher<geometry_msgs::msg::Point>("current_pose", 100);
	next_pose_pub = node->create_publisher<geometry_msgs::msg::Point>("next_pose", 100);
	lidar_pub = node->create_publisher<geometry_msgs::msg::Point>("lidar_scan", 100);
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

pose_t Autonomous::poseToDraw(pose_t &pose, pose_t &current_pose) const
{
	// Both poses are given in the same frame. (usually GPS frame)
	// `current_pose` is the current location of the robot, `pose` is the pose we wish to draw
	transform_t inv_curr = toTransform(current_pose).inverse();
	return toPose(toTransform(pose) * inv_curr * VIZ_BASE_TF, 0);
}

void Autonomous::publish(Eigen::Vector3d pose, rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr &publisher) const
{
	auto message = geometry_msgs::msg::Point();
	message.x = pose(0);
	message.y = pose(1);
	message.z = pose(2);
	publisher->publish(message);
}

bool Autonomous::arrived(const pose_t &pose) const
{
	if (landmarkFilter.getSize() == 0)
	{
		return false;
	}
	pose_t target = getTargetPose();
	target.topRows(2) = landmarkFilter.get().topRows(2);
	return dist(pose, target, 1.0) < 0.5;
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
		pose_t sum = pose_t::Zero();
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

	if (already_arrived || arrived(pose))
	{
		already_arrived = true;
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
				if (landmarkFilter.getSize() == 0)
				{
					// Replan if this is the first landmark we have seen
					plan_cost = INFINITE_COST;
				}
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
		// If the target has a second post we can see and we haven't seen the first post yet,
		// use the second post as the drive target without adding it to the filter
		else if (target.right_post_id != -1 && landmarks[target.right_post_id](2) != 0)
		{
			point_t rightPostLandmark = landmarks[target.right_post_id];
			// transform and add the new data to the filter
			transform_t invTransform = toTransform(pose).inverse();
			point_t landmarkMapSpace = invTransform * rightPostLandmark;
			// the filtering is done on the target in map space to reduce any phase lag
			// caused by filtering
			driveTarget.topRows(2) = landmarkFilter.get(landmarkMapSpace).topRows(2);
			// clear the filter so the second post is removed from it
			landmarkFilter.reset();
		}
		// If we are in a search pattern, set the drive target to the next search point
		else if (state == NavState::SEARCH_PATTERN)
		{
			driveTarget = search_target;
		}

		const points_t lidar_scan = readLidarScan();
		if (plan_cost == INFINITE_COST || ++time_since_plan % REPLAN_PERIOD == 0) {
			point_t point_t_goal;
			point_t_goal.topRows(2) = driveTarget.topRows(2);
			point_t_goal(2) = 1.0;
			point_t_goal = toTransform(pose) * point_t_goal;
			double goal_radius = 2.0;
			plan_t new_plan = getPlan(lidar_scan, point_t_goal, goal_radius);
			double new_plan_cost = planCostFromIndex(new_plan, 0);
			// we want a significant improvement to avoid thrash
			if (new_plan_cost < plan_cost * 0.8) {
				plan_idx = 0;
				plan_base = pose;
				plan_cost = new_plan_cost;
				plan = new_plan;
				time_since_plan = 0;
			}
		}

		// Send message to visualization to clear previous data
		publish({NAN, NAN, NAN}, plan_pub);

		// Send lidar points to visualization
		const points_t lidar_scan_transformed = transformReadings(lidar_scan, VIZ_BASE_TF);
		for (point_t point : lidar_scan_transformed)
		{
			publish(point, lidar_pub);
		}

		// Send current pose to visualization
		const pose_t curr_pose_transformed = poseToDraw(pose, pose);
		publish(curr_pose_transformed, curr_pose_pub);

		int plan_size = plan.rows();
		if (plan_size == 0 && dist(driveTarget, pose, 1.0) < 2.0) {
			// We're probably within planning resolution of the goal,
			// so using the goal as the drive target makes sense.
		} else {
			// Roll out the plan until we find a target pose a certain distance in front
			// of the robot. (This code also handles visualizing the plan.)
			pose_t plan_pose = plan_base;

			// Send starting plan_pose
			const pose_t start_plan_pose_transformed = poseToDraw(plan_pose, pose);
			publish(start_plan_pose_transformed, plan_pub);

			bool found_target = false;
			transform_t lidar_base_inv = toTransform(pose).inverse();
			for (int i = 0; i < plan_size; i++) {
				action_t action = plan.row(i);
				plan_pose(2) += action(0);
				plan_pose(0) += action(1) * cos(plan_pose(2));
				plan_pose(1) += action(1) * sin(plan_pose(2));
				transform_t tf_plan_pose = toTransform(plan_pose) * lidar_base_inv;
				if (collides(tf_plan_pose, lidar_scan, 1.3)) { // stay 1.3 meters away
					// We'll replan next timestep
					// TODO strictly speaking, we don't need to replan if the collision happened
					// on a part of the plan that we've already passed. But I'm not quite sure how
					// to algorithmically decide which parts of the plan we've already passed.
					plan_cost = INFINITE_COST;
					// TODO we should have a more sophisticated way of deciding whether a collision
					// is imminent.
					if (dist(plan_pose, pose, 0.0) < PLAN_COLLISION_STOP_DIST) {
						driveTarget = pose; // Don't move
					}
				}
				if (i >= plan_idx && !found_target && dist(plan_pose, pose, 1.0) > FOLLOW_DIST) {
					found_target = true;
					plan_idx = i;
					plan_cost = planCostFromIndex(plan, i);
					driveTarget = plan_pose;
					// Send next target pose to visualization
					const pose_t next_pose_transformed = poseToDraw(plan_pose, pose);
					publish(next_pose_transformed, next_pose_pub);
				} else {
					// Send current plan_pose to visualization
					const pose_t curr_plan_transformed = poseToDraw(plan_pose, pose);
					publish(curr_plan_transformed, plan_pub);
				}
			}
		}

		// Send message to visualization that all data has been sent
		publish({INFINITY, INFINITY, INFINITY}, plan_pub);

		double d = dist(driveTarget, pose, 0);
		if (state == NavState::SEARCH_PATTERN && dist(search_target, pose, 0.0) < 0.5)
		{
			// Current search point has been reached
			// Replan to avoid waiting at current search point
			plan_cost = INFINITE_COST;
			// Set the search target to the next point in the search pattern
			search_target -= target.approx_GPS;
			double radius = hypot(search_target(0), search_target(1));
			double scale = (radius + search_theta_increment) / radius;
			// Rotate the target counterclockwise by the theta increment
			search_target.topRows(2) =
				(Eigen::Matrix2d() << cos(search_theta_increment), -sin(search_theta_increment),
										sin(search_theta_increment), cos(search_theta_increment)).finished()
				* search_target.topRows(2) * scale;
			search_target += target.approx_GPS;
			// Adjust the target angle
			search_target(2) += search_theta_increment;
			if (search_target(2) > PI)
			{
				search_target(2) -= 2 * PI;
				// Decrease the theta increment every time a full rotation is made so the
				// spiral shape is followed better
				// This increases the denominator of the theta increment by 4
				search_theta_increment = PI / ((int) (PI / search_theta_increment + 4));
			}
		}
		if (state != NavState::SEARCH_PATTERN && dist(target.approx_GPS, pose, 0.0) < 0.2 &&
			landmarkFilter.getSize() == 0 && !landmarkVisible)
		{
			// Close to GPS target but no landmark in sight, should use search pattern
			// Replan so the search starts faster
			plan_cost = INFINITE_COST;
			state = NavState::SEARCH_PATTERN;
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
	pose_t ret{target.approx_GPS(0), target.approx_GPS(1), 0.0};
	return ret;
}
