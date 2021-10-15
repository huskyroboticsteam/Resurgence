#include "Autonomous.h"

#include "Globals.h"
#include "commands/DriveToGateNoCompass.h"
#include "commands/nogps/DriveThroughGate.h"
#include "commands/nogps/DriveToGate.h"
#include "log.h"
#include "rospub.h"
#include "simulator/constants.h"
#include "world_interface/world_interface.h"

#include <cmath>
#include <iostream>

#include <Eigen/LU>

constexpr float PI = M_PI;
constexpr double KP_ANGLE = 2.0;
constexpr double FAST_DRIVE_SPEED = 1.5;
constexpr double CAREFUL_DRIVE_SPEED = 1.0;
const Eigen::Vector3d gpsStdDev = {2, 2, 3};
constexpr int numSamples = 1;

constexpr double FOLLOW_DIST = 2.0;
constexpr double PLANNING_GOAL_REGION_RADIUS = 1.0;
constexpr double PLAN_COLLISION_DIST = 1.3;
constexpr double PLAN_COLLISION_STOP_DIST = 4.0;
constexpr double INFINITE_COST = 1e10;
constexpr int REPLAN_PERIOD = 20;

constexpr bool USE_MAP = false;

constexpr bool LEFT = true;
constexpr bool RIGHT = false;

constexpr auto ZERO_DURATION = std::chrono::microseconds(0);

const transform_t VIZ_BASE_TF =
	toTransform({NavSim::DEFAULT_WINDOW_CENTER_X, NavSim::DEFAULT_WINDOW_CENTER_Y, M_PI / 2});

/**
   @brief Prints a list of landmarks using the logging mechanism.

   The message is formatted as "Landmarks: [i: {x, y, z}, ...]" where i is the index of the
   landmark in the list.

   @param landmarks The list of landmarks to print.
   @param log_level The optional log level to use; defaults to LOG_DEBUG so the message isn't
   seen if on a higher logging level.
 */
static void printLandmarks(points_t& landmarks, int log_level = LOG_DEBUG);

Autonomous::Autonomous(const std::vector<URCLeg>& _targets, double controlHz)
	: urc_targets(_targets), leg_idx(5),
	  search_target({_targets[0].approx_GPS(0) - PI, _targets[0].approx_GPS(1), -PI / 2}),
	  gate_targets({NAN, NAN, NAN}, {NAN, NAN, NAN}), gate_direction(true),
	  poseEstimator({0.2, 0.2}, gpsStdDev, Constants::WHEEL_BASE, 1.0 / controlHz),
	  map(10000), // TODO: the stride should be set higher when using the real lidar
	  calibrated(false), calibrationPoses({}), nav_state(NavState::GPS),
	  control_state(ControlState::FAR_FROM_TARGET_POSE), time_since_plan(0), plan(0, 2),
	  pending_plan(), pending_solve(), plan_cost(INFINITE_COST), plan_base({0, 0, 0}),
	  plan_idx(0), search_theta_increment(PI / 4), mapLoopCounter(0), mapBlindPeriod(15),
	  mapDoesOverlap(false), mapOverlapSampleThreshold(15),
	  pose_graph(0, 0, 0, 0, 0), // We'll re-initialize this in the function body
	  pose_id(0), prev_odom(toTransform({0, 0, 0})), smoothed_traj({}),
	  smoothed_landmarks({}) {

	int num_landmarks = 0;
	for (auto target : _targets) {
		int num_landmarks_for_target = (target.right_post_id == -1) ? 1 : 2;
		num_landmarks += num_landmarks_for_target;
	}
	// We use a gps_xy_std that's larger than the true std because empirically
	// that leads to smoother changes in the pose estimate (at least in the simulator),
	// which are easier for the controller to work with.
	pose_graph = FriendlyGraph(num_landmarks, 10, 0.3, 5.0, 0.05), prev_odom = readOdom();
	transform_t start_pose_guess = toTransform({13, -1, M_PI * 2 / 3});
	double pose_prior_xy_std = 3.0;
	double pose_prior_th_std = 1.0;
	covariance<3> pose_prior_cov = covariance<3>::Zero();
	pose_prior_cov << pose_prior_xy_std * pose_prior_xy_std, 0, 0, 0,
		pose_prior_xy_std * pose_prior_xy_std, 0, 0, 0, pose_prior_th_std * pose_prior_th_std;
	pose_graph.addPosePrior(0, start_pose_guess, pose_prior_cov);
	double lm_std = 5.0;
	for (auto target : _targets) {
		pose_graph.addLandmarkPrior(target.left_post_id, target.approx_GPS, lm_std);
		if (target.right_post_id != -1) {
			pose_graph.addLandmarkPrior(target.right_post_id, target.approx_GPS, lm_std);
		}
	}
	pose_graph.solve();
	smoothed_traj = pose_graph.getSmoothedTrajectory();
	smoothed_landmarks = pose_graph.getLandmarkLocations();
}

Autonomous::Autonomous(const std::vector<URCLeg>& _targets, double controlHz,
					   const pose_t& startPose)
	: Autonomous(_targets, controlHz) {
	poseEstimator.reset(startPose);
	calibrated = true;
}

double dist(const pose_t& p1, const pose_t& p2, double theta_weight) {
	pose_t diff = p1 - p2;
	// angles are modular in nature, so wrap at 2pi radians
	double thetaDiff = std::fmod(fabs(diff(2)), 2 * PI);
	// change domain from [0, 2pi) to (-pi, pi]
	if (thetaDiff > PI) {
		thetaDiff -= 2 * PI;
	}
	diff(2) = thetaDiff * theta_weight;
	return diff.norm();
}

void Autonomous::endCurrentLeg() {
	log(LOG_INFO, "Leg complete, exiting autonomous mode\n");
	setCmdVel(0, 0);
	Globals::AUTONOMOUS = false;
	leg_idx += 1;
	// clear the cached data points

	if (leg_idx == urc_targets.size()) {
		log(LOG_INFO, "All legs complete\n");
	} else {
		search_target = {urc_targets[leg_idx].approx_GPS(0) - PI,
						 urc_targets[leg_idx].approx_GPS(1), -PI / 2};
		setNavState(NavState::GPS);
		log(LOG_INFO, "Enter autonomous mode to start next leg\n");
	}
}

void Autonomous::setNavState(NavState s) {
	NavState old_state = nav_state;
	nav_state = s;
	if (old_state != s) {
		log(LOG_INFO, "Changing navigation state to %s\n", NAV_STATE_NAMES[nav_state]);
		plan_cost = INFINITE_COST;
	}
}

pose_t Autonomous::poseToDraw(const pose_t& pose, const pose_t& current_pose) const {
	// Both poses are given in the same frame. (usually GPS frame)
	// `current_pose` is the current location of the robot, `pose` is the pose we wish to draw
	transform_t inv_curr = toTransform(current_pose).inverse();
	return toPose(toTransform(pose) * inv_curr * VIZ_BASE_TF, 0);
}

void Autonomous::update_nav_state(const pose_t& pose, const pose_t& plan_target) {
	if (nav_state == NavState::GPS || nav_state == NavState::SEARCH_PATTERN ||
		nav_state == NavState::SEARCH_PATTERN_SECOND_POST ||
		nav_state == NavState::POST_VISIBLE) {
		bool foundLeftPost = getPostVisibility(LEFT);
		bool foundRightPost = getPostVisibility(RIGHT);
		if (foundLeftPost && foundRightPost) {
			setNavState(NavState::GATE_ALIGN);
		} else if ((foundLeftPost || foundRightPost) &&
				   (nav_state != SEARCH_PATTERN_SECOND_POST)) {
			setNavState(NavState::POST_VISIBLE);
		}
	}

	if (dist(pose, plan_target, 1.0) < 0.5) {
		if (nav_state == NavState::GPS) {
			log(LOG_WARN, "Reached GPS target without seeing post!\n");
			setNavState(NavState::SEARCH_PATTERN);
		} else if (nav_state == NavState::POST_VISIBLE) {
			if (urc_targets[leg_idx].right_post_id == -1) {
				log(LOG_INFO, "Arrived at post\n");
				setNavState(NavState::DONE);
			} else {
				log(LOG_WARN, "Reached left post without seeing right post!\n");
				setNavState(NavState::SEARCH_PATTERN_SECOND_POST);
			}
		} else if (nav_state == NavState::GATE_ALIGN) {
			setNavState(NavState::GATE_TRAVERSE);
		} else if (nav_state == NavState::GATE_TRAVERSE) {
			log(LOG_INFO, "Arrived at gate\n");
			setNavState(NavState::DONE);
		} else if (nav_state == NavState::SEARCH_PATTERN ||
				   nav_state == NavState::SEARCH_PATTERN_SECOND_POST) {
			// Current search point has been reached
			updateSearchTarget();
		}
	}
}

bool calibratePeriodic(std::vector<pose_t>& poses, const pose_t& pose, pose_t& out) {
	poses.push_back(pose);
	if (poses.size() == numSamples) {
		pose_t sum = pose_t::Zero();
		for (const pose_t& p : poses) {
			sum += p;
		}
		sum /= poses.size();
		out = sum;
		return true;
	}
	return false;
}

double getRelativeAngle(double currAngle, double targetAngle) {
	double range = 2 * PI;
	double dist = std::fmod(targetAngle - currAngle, range);
	double absDist = fabs(dist);

	int sign = dist > 0 ? 1 : (dist < 0 ? -1 : 0);

	return currAngle + ((absDist > (range / 2)) ? -sign * (range - absDist) : dist);
}

double Autonomous::getLinearVel(const pose_t& drive_target, const pose_t& pose,
								double thetaErr) const {
	double speed = FAST_DRIVE_SPEED;
	if (dist(drive_target, pose, 0) < 1.0 || nav_state == NavState::GATE_TRAVERSE) {
		// We may need to drive slower near the goal so that we don't overshoot the target
		// (given our relatively low control frequency of 10 Hz)
		speed = CAREFUL_DRIVE_SPEED;
	}
	if (fabs(thetaErr) > PI / 4 || control_state == ControlState::NEAR_TARGET_POSE) {
		// don't drive forward if pointing away
		// or if we're already very close to the target
		speed = 0;
	}
	return speed;
}

double Autonomous::getThetaVel(const pose_t& drive_target, const pose_t& pose,
							   double& thetaErr) const {
	// If we're within 20cm of the target location, we want to turn the rover until
	// we reach the target orientation. Otherwise, we want to turn the rover to
	// aim it at the target location.
	double targetAngle = drive_target(2);
	if (control_state == ControlState::FAR_FROM_TARGET_POSE) {
		double dx = drive_target(0) - pose(0);
		double dy = drive_target(1) - pose(1);
		targetAngle = atan2(dy, dx);
	}
	double relativeTargetAngle = getRelativeAngle(pose(2), targetAngle);
	thetaErr = relativeTargetAngle - pose(2);

	return KP_ANGLE * thetaErr;
}

int Autonomous::getPostID(bool left) {
	if (urc_targets[leg_idx].left_post_id < 0 ||
		static_cast<unsigned>(urc_targets[leg_idx].left_post_id) > smoothed_landmarks.size()) {
		log(LOG_ERROR, "Invalid left_post_id %d\n", urc_targets[leg_idx].left_post_id);
		return 0;
	}
	int post_id =
		left ? urc_targets[leg_idx].left_post_id : urc_targets[leg_idx].right_post_id;
	return post_id;
}

point_t Autonomous::getPostLocation(bool left) {
	return smoothed_landmarks[getPostID(left)];
}

bool Autonomous::getPostVisibility(bool left) {
	// This is a bit of a hack. We decide whether a post is currently visible based
	// on whether its current estimated location is different from its approximate
	// GPS coordinates. Really what we should do is implement some pose graph function
	// that tells us how uncertain we are about the location of the post, but that's
	// more complicated to implement.
	point_t prior_location = urc_targets[leg_idx].approx_GPS;
	point_t location = getPostLocation(left);
	double diff = (location - prior_location).norm();
	bool different_from_prior = (diff > 0.001);
	log(LOG_DEBUG,
		"Visibility %d is %d: diff %.3f loc (%.3f %.3f %.3f) prior (%.3f %.3f %.3f)\n", left,
		different_from_prior, diff, location(0), location(1), location(2), prior_location(0),
		prior_location(1), prior_location(2));
	return different_from_prior;
}

pose_t Autonomous::choose_plan_target(const pose_t& pose) {
	// When traversing the gate, we don't want to reverse the order of the gate targets
	computeGateTargets(pose, nav_state != NavState::GATE_TRAVERSE);

	if (nav_state == NavState::GPS) {
		return getGPSTargetPose();
	} else if (nav_state == NavState::SEARCH_PATTERN ||
			   nav_state == NavState::SEARCH_PATTERN_SECOND_POST) {
		return search_target;
	} else if (nav_state == NavState::POST_VISIBLE) {
		bool post_to_get = getPostVisibility(LEFT) ? LEFT : RIGHT;
		point_t post = getPostLocation(post_to_get);
		double angle = std::atan2(post(1) - pose(1), post(0) - pose(0));
		// Offset the target by two meters to avoid crashing into the post
		pose_t plan_target({post(0) - 2.0 * cos(angle), post(1) - 2.0 * sin(angle), angle});
		return plan_target;
	} else if (nav_state == NavState::GATE_ALIGN) {
		return gate_targets.first;
	} else if (nav_state == NavState::GATE_TRAVERSE) {
		return gate_targets.second;
	} else // probably NavState::DONE
	{
		return {0, 0, 0};
	}
}

void Autonomous::computeGateTargets(const pose_t& pose, bool choose_direction) {
	if (!(getPostVisibility(LEFT) && getPostVisibility(RIGHT)))
		return;

	point_t post_1 = getPostLocation(LEFT);
	point_t post_2 = getPostLocation(RIGHT);
	point_t center = (post_1 + post_2) / 2;

	point_t offset{(post_1(1) - post_2(1)) / 2, (post_2(0) - post_1(0)) / 2, 0};
	pose_t goal_1 = center + 2 * offset;
	pose_t goal_2 = center - 2 * offset;

	// Choose the closest goal to be the first target
	if (choose_direction) {
		gate_direction = (dist(pose, goal_1, 1.0) < dist(pose, goal_2, 1.0));
	}

	if (gate_direction == true) {
		gate_targets = std::make_pair(goal_1, goal_2);
	} else {
		gate_targets = std::make_pair(goal_2, goal_1);
	}

	// Angle between targets
	double angle = std::atan2(gate_targets.second(1) - gate_targets.first(1),
							  gate_targets.second(0) - gate_targets.first(0));
	gate_targets.first(2) = angle;
	gate_targets.second(2) = angle;
}

void Autonomous::updateSearchTarget() {
	// Replan to avoid waiting at current search point
	plan_cost = INFINITE_COST;
	// Set the search target to the next point in the search pattern
	search_target -= urc_targets[leg_idx].approx_GPS;
	double radius = hypot(search_target(0), search_target(1));
	double scale = (radius + search_theta_increment) / radius;
	// Rotate the target counterclockwise by the theta increment
	search_target.topRows(2) =
		(Eigen::Matrix2d() << cos(search_theta_increment), -sin(search_theta_increment),
		 sin(search_theta_increment), cos(search_theta_increment))
			.finished() *
		search_target.topRows(2) * scale;
	search_target += urc_targets[leg_idx].approx_GPS;
	// Adjust the target angle
	search_target(2) += search_theta_increment;
	if (search_target(2) > PI) {
		search_target(2) -= 2 * PI;
		// Decrease the theta increment every time a full rotation is made so the
		// spiral shape is followed better
		// This increases the denominator of the theta increment by 4
		search_theta_increment = PI / ((int)(PI / search_theta_increment + 4));
	}
}

plan_t computePlan(transform_t invTransform, point_t goal) {
	// We need to readLidar again from within the planning thread, for thread safety
	points_t lidar_scan = readLidarScan();
	auto collide_func = [&](double x, double y, double radius) -> bool {
		// transform the point to check into map space
		point_t relPoint = {x, y, 1};
		point_t p = invTransform * relPoint;
		transform_t coll_tf = toTransform(relPoint);
		// TODO implement thread-safe access to `map` if `USE_MAP` is true
		// if (USE_MAP) return map.hasPointWithin(p, radius);
		return collides(coll_tf, lidar_scan, radius);
	};
	return getPlan(collide_func, goal, PLANNING_GOAL_REGION_RADIUS);
}

transform_t Autonomous::optimizePoseGraph(transform_t current_gps, transform_t current_odom) {
	if (current_odom != prev_odom) { // Don't add poses to pose graph if we haven't moved
		pose_id += 1;
		pose_graph.addOdomMeasurement(pose_id, pose_id - 1, current_odom, prev_odom);
	}

	points_t landmarks = readLandmarks();
	printLandmarks(landmarks);
	for (size_t lm_id = 0; lm_id < landmarks.size(); lm_id++) {
		point_t lm = landmarks[lm_id];
		if (lm(2) != 0.0)
			pose_graph.addLandmarkMeasurement(pose_id, (int)lm_id, lm);
	}
	pose_graph.addGPSMeasurement(pose_id, current_gps);
	pose_graph.solve();

	return current_odom;
}

void Autonomous::autonomyIter() {
	transform_t gps = readGPS();
	transform_t odom = readOdom();

	if ((gps.norm() != 0.0) && !pending_solve.valid()) {
		// TODO even if we already have a pose graph optimization running, we should still
		// add this GPS measurement to the pose graph. However, that situation should never
		// occur as long as pose graph solves take significantly less time than one second.
		pending_solve =
			std::async(std::launch::async, &Autonomous::optimizePoseGraph, this, gps, odom);
	}

	if (pending_solve.valid() &&
		pending_solve.wait_for(ZERO_DURATION) == std::future_status::ready) {
		prev_odom = pending_solve.get();
		smoothed_traj = pose_graph.getSmoothedTrajectory();
		smoothed_landmarks = pose_graph.getLandmarkLocations();
	}

	transform_t current_tf =
		odom * prev_odom.inverse() * smoothed_traj[smoothed_traj.size() - 1];
	pose_t pose = toPose(current_tf, 0.0);
	log(LOG_DEBUG, "Pose %f %f %f\n", pose(0), pose(1), pose(2));

	for (transform_t& sm_tf : smoothed_traj) {
		const pose_t sm_tf_transformed = poseToDraw(toPose(sm_tf, 0.0), pose);
		rospub::publish(sm_tf_transformed, rospub::PointPub::POSE_GRAPH);
	}

	transform_t invTransform = toTransform(pose).inverse();

	const points_t landmarks_transformed =
		transformReadings(smoothed_landmarks, invTransform * VIZ_BASE_TF);
	rospub::publish_array(landmarks_transformed, rospub::ArrayPub::LANDMARKS);

	if (nav_state == NavState::DONE && Globals::AUTONOMOUS) {
		endCurrentLeg();
		return;
	}

	pose_t plan_target = choose_plan_target(pose);
	update_nav_state(pose, plan_target);

	points_t lidar_scan = readLidarScan();

	if (USE_MAP) {
		if (mapLoopCounter++ > mapBlindPeriod) {
			map.addPoints(toTransform(pose), lidar_scan, mapDoesOverlap ? 0.4 : 0);
			mapDoesOverlap = lidar_scan.size() > mapOverlapSampleThreshold;
		}
	}

	// if we don't have a plan pending, we should start computing one.
	if (!pending_plan.valid()) {
		point_t point_t_goal;
		point_t_goal.topRows(2) = plan_target.topRows(2);
		point_t_goal(2) = 1.0;
		point_t_goal = toTransform(pose) * point_t_goal;
		pending_plan =
			std::async(std::launch::async, &computePlan, invTransform, point_t_goal);
	}
	// if there is a plan pending, check if it is ready.
	else if (pending_plan.wait_for(ZERO_DURATION) == std::future_status::ready) {
		// plan is ready; retrieve it and check if the cost is satisfactory.
		// upon calling get(), pending_plan.valid() will return false, so if we reject this
		// plan another will be computed.
		plan_t new_plan = pending_plan.get();
		double new_plan_cost = planCostFromIndex(new_plan, 0);
		// we want a significant improvement to avoid thrash
		log(LOG_DEBUG, "old cost %f, new cost %f\n", plan_cost, new_plan_cost);
		if (new_plan_cost < plan_cost * 0.8) {
			plan_idx = 0;
			plan_base = pose;
			plan_cost = new_plan_cost;
			plan = new_plan;
			time_since_plan = 0;
		}
	}

	// Send lidar points to visualization
	const points_t lidar_scan_transformed = transformReadings(lidar_scan, VIZ_BASE_TF);
	log(LOG_DEBUG, "Publishing %d lidar points\n", lidar_scan_transformed.size());
	rospub::publish_array(lidar_scan_transformed, rospub::ArrayPub::LIDAR_SCAN);

	// Send current pose and plan target to visualization
	const pose_t curr_pose_transformed = poseToDraw(pose, pose);
	rospub::publish(curr_pose_transformed, rospub::PointPub::CURRENT_POSE);
	const pose_t plan_target_transformed = poseToDraw(plan_target, pose);
	rospub::publish(plan_target_transformed, rospub::PointPub::PLAN_TARGET);

	int plan_size = plan.rows();
	pose_t driveTarget = pose;
	if (nav_state == NavState::GATE_TRAVERSE || dist(plan_target, pose, 0.0) < 2.0) {
		// We're probably within planning resolution of the goal,
		// so using the goal as the drive target makes sense.
		driveTarget = plan_target;
	} else {
		// Roll out the plan until we find a target pose a certain distance in front
		// of the robot. (This code also handles visualizing the plan.)
		pose_t plan_pose = plan_base;

		// Send starting plan_pose
		const pose_t start_plan_pose_transformed = poseToDraw(plan_pose, pose);
		rospub::publish(start_plan_pose_transformed, rospub::PointPub::PLAN_VIZ);

		bool found_target = false;
		double accumulated_cost = 0;
		for (int i = 0; i < plan_size; i++) {
			action_t action = plan.row(i);
			plan_pose(2) += action(0);
			plan_pose(0) += action(1) * cos(plan_pose(2));
			plan_pose(1) += action(1) * sin(plan_pose(2));
			transform_t tf_plan_pose = toTransform(plan_pose) * invTransform;
			// We don't care about collisions on already-executed parts of the plan
			bool plan_pose_collides = (i >= plan_idx);
			if (USE_MAP) {
				plan_pose_collides &= map.hasPointWithin(plan_pose, PLAN_COLLISION_DIST);
			} else {
				plan_pose_collides &= collides(tf_plan_pose, lidar_scan, PLAN_COLLISION_DIST);
			}
			if (plan_pose_collides) {
				// We'll replan next timestep
				accumulated_cost += INFINITE_COST;
				// TODO we should have a more sophisticated way of deciding whether a collision
				// is imminent.
				if (dist(plan_pose, pose, 0.0) < PLAN_COLLISION_STOP_DIST) {
					log(LOG_WARN, "Collision imminent! Stopping the rover!\n");
					driveTarget = pose; // Don't move
				}
			}
			if (i >= plan_idx && !found_target && dist(plan_pose, pose, 1.0) > FOLLOW_DIST) {
				found_target = true;
				plan_idx = i;
				accumulated_cost += planCostFromIndex(plan, i);
				driveTarget = plan_pose;
			} else {
				// Send current plan_pose to visualization
				const pose_t curr_plan_transformed = poseToDraw(plan_pose, pose);
				rospub::publish(curr_plan_transformed, rospub::PointPub::PLAN_VIZ);
			}
		}
		if (!found_target) {
			plan_idx = plan_size - 1;
			driveTarget = plan_pose;
		}

		// This deals with drift
		// TODO maybe we should just replan if the distance drifts too far
		accumulated_cost += 2 * dist(plan_pose, plan_target, 1.0);
		plan_cost = accumulated_cost;
	}

	// Send drive target to visualization
	const pose_t drive_target_transformed = poseToDraw(driveTarget, pose);
	rospub::publish(drive_target_transformed, rospub::PointPub::DRIVE_TARGET);

	// Send message to visualization that all data has been sent
	rospub::publish({NAN, NAN, NAN}, rospub::PointPub::PLAN_VIZ);

	double d = dist(driveTarget, pose, 0);
	// There's an overlap where either state might apply, to prevent rapidly switching
	// back and forth between these two states.
	if (d < 0.2 && control_state == ControlState::FAR_FROM_TARGET_POSE) {
		control_state = ControlState::NEAR_TARGET_POSE;
	}
	if (d > 0.5 && control_state == ControlState::NEAR_TARGET_POSE) {
		control_state = ControlState::FAR_FROM_TARGET_POSE;
	}
	double thetaErr;
	double thetaVel = getThetaVel(driveTarget, pose, thetaErr);
	double driveSpeed = getLinearVel(driveTarget, pose, thetaErr);

	if (!Globals::E_STOP && Globals::AUTONOMOUS) {
		double scale_factor = setCmdVel(thetaVel, driveSpeed);
	}
}

pose_t Autonomous::getGPSTargetPose() const {
	pose_t ret{urc_targets[leg_idx].approx_GPS(0), urc_targets[leg_idx].approx_GPS(1), 0.0};
	return ret;
}

static void printLandmarks(points_t& landmarks, int log_level) {
	std::ostringstream stream;
	stream << "Landmarks: [";
	for (size_t i = 0; i < landmarks.size(); i++) {
		stream << i << ": {" << landmarks[i][0] << ", " << landmarks[i][1] << ", "
			   << landmarks[i][2] << "}";
		if (i < landmarks.size() - 1) {
			stream << ", ";
		}
	}
	stream << "}" << std::endl;
	log(log_level, stream.str().c_str());
}
