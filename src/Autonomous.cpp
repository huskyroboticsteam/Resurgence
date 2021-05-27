#include "Autonomous.h"

#include <Eigen/LU>
#include <cmath>
#include <iostream>

#include "Globals.h"
#include "log.h"
#include "simulator/world_interface.h"
#include "simulator/constants.h"

constexpr float PI = M_PI;
constexpr double KP_ANGLE = 2;
constexpr double DRIVE_SPEED = 3;
const Eigen::Vector3d gpsStdDev = {2, 2, PI / 24};
constexpr int numSamples = 1;

constexpr double FOLLOW_DIST = 2.0;
constexpr double PLANNING_GOAL_REGION_RADIUS = 1.0;
constexpr double PLAN_COLLISION_STOP_DIST = 4.0;
constexpr double INFINITE_COST = 1e10;
constexpr int REPLAN_PERIOD = 20;

constexpr const char * ROVER_FRAME = "rover";

const transform_t VIZ_BASE_TF = toTransform({NavSim::DEFAULT_WINDOW_CENTER_X,NavSim::DEFAULT_WINDOW_CENTER_Y,M_PI/2});

/**
   @brief Prints a list of landmarks using the logging mechanism.

   The message is formatted as "Landmarks: [i: {x, y, z}, ...]" where i is the index of the
   landmark in the list.

   @param landmarks The list of landmarks to print.
   @param log_level The optional log level to use; defaults to LOG_DEBUG so the message isn't
   seen if on a higher logging level.
 */
static void printLandmarks(points_t& landmarks, int log_level = LOG_DEBUG);

Autonomous::Autonomous(const URCLeg &_target, double controlHz)
	: Node("autonomous"),
		urc_target(_target),
		search_target({_target.approx_GPS(0) - PI, _target.approx_GPS(1), -PI / 2}),
		gate_targets({NAN, NAN, NAN}, {NAN, NAN, NAN}),
		poseEstimator({1.5, 1.5}, gpsStdDev, Constants::WHEEL_BASE, 1.0 / controlHz),
		calibrated(false),
		calibrationPoses({}),
		leftPostFilter(),
		rightPostFilter(),
		nav_state(NavState::GPS),
		control_state(ControlState::FAR_FROM_TARGET_POSE),
		time_since_plan(0),
		plan(0,2),
		plan_cost(INFINITE_COST),
		plan_base({0,0,0}),
		plan_idx(0),
		search_theta_increment(PI / 4),
		plan_pub(this->create_publisher<geometry_msgs::msg::Point>("plan_viz", 100)),
		curr_pose_pub(this->create_publisher<geometry_msgs::msg::Point>("current_pose", 100)),
		drive_target_pub(this->create_publisher<geometry_msgs::msg::Point>("drive_target", 100)),
		plan_target_pub(this->create_publisher<geometry_msgs::msg::Point>("plan_target", 100)),
		lidar_pub(this->create_publisher<geometry_msgs::msg::PoseArray>("lidar_scan", 100)),
		landmarks_pub(this->create_publisher<geometry_msgs::msg::PoseArray>("landmarks", 100))
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

void Autonomous::setNavState(NavState s)
{
  NavState old_state = nav_state;
  nav_state = s;
  if (old_state != s)
  {
    log(LOG_INFO, "Changing navigation state to %s\n", NAV_STATE_NAMES[nav_state]);
    plan_cost = INFINITE_COST;
  }
}

pose_t Autonomous::poseToDraw(pose_t &pose, pose_t &current_pose) const
{
	// Both poses are given in the same frame. (usually GPS frame)
	// `current_pose` is the current location of the robot, `pose` is the pose we wish to draw
	transform_t inv_curr = toTransform(current_pose).inverse();
	return toPose(toTransform(pose) * inv_curr * VIZ_BASE_TF, 0);
}

void Autonomous::publish(Eigen::Vector3d pose,
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr &publisher) const
{
	auto message = geometry_msgs::msg::Point();
	message.x = pose(0);
	message.y = pose(1);
	message.z = pose(2);
	publisher->publish(message);
}

void Autonomous::publish_array(const points_t &points,
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr &publisher) const
{
	auto message = geometry_msgs::msg::PoseArray();
	message.header.frame_id = ROVER_FRAME; // technically this is in the window frame actually
	// but we'll figure out our transform situation later
	for (point_t l : points) {
		auto p = geometry_msgs::msg::Pose();
		p.position.x = l(0);
		p.position.y = l(1);
		p.position.z = l(2);
		message.poses.push_back(p);
	}
	publisher->publish(message);
}

void Autonomous::update_nav_state(const pose_t &pose, const pose_t &plan_target) {
  if (nav_state == NavState::GPS ||
      nav_state == NavState::SEARCH_PATTERN ||
      nav_state == NavState::SEARCH_PATTERN_SECOND_POST ||
      nav_state == NavState::POST_VISIBLE)
  {
    bool foundLeftPost = (leftPostFilter.getSize() > 0);
    bool foundRightPost = (rightPostFilter.getSize() > 0);
    if (foundLeftPost && foundRightPost) {
      setNavState(NavState::GATE_ALIGN);
    } else if ((foundLeftPost || foundRightPost) && (nav_state != SEARCH_PATTERN_SECOND_POST)) {
      setNavState(NavState::POST_VISIBLE);
    }
  }

  if (dist(pose, plan_target, 1.0) < 0.5) {
    if (nav_state == NavState::GPS) {
      log(LOG_WARN, "Reached GPS target without seeing post!\n");
      setNavState(NavState::SEARCH_PATTERN);
    } else if (nav_state == NavState::POST_VISIBLE) {
      if (urc_target.right_post_id == -1) {
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
	return false;
}

double getRelativeAngle(double currAngle, double targetAngle)
{
	double range = 2 * PI;
	double dist = std::fmod(targetAngle - currAngle, range);
	double absDist = abs(dist);

	int sign = dist > 0 ? 1 : (dist < 0 ? -1 : 0);

	return currAngle + ((absDist > (range / 2)) ? -sign * (range - absDist) : dist);
}

double Autonomous::getLinearVel(const pose_t &drive_target, const pose_t &pose, double thetaErr) const {
	double speed = DRIVE_SPEED;
	if (dist(drive_target, pose, 0) < 1.0 ||
			nav_state == NavState::GATE_TRAVERSE) {
		// We should drive slower near the goal so that we don't overshoot the target
		// (given our relatively low control frequency of 10 Hz)
		speed = DRIVE_SPEED / 3;
	}
	if (abs(thetaErr) > PI / 4 || control_state == ControlState::NEAR_TARGET_POSE) {
		// don't drive forward if pointing away
		// or if we're already very close to the target
		speed = 0;
	}
	return speed;
}

double Autonomous::getThetaVel(const pose_t &drive_target, const pose_t &pose, double &thetaErr) const
{
	// If we're within 20cm of the target location, we want to turn the rover until
	// we reach the target orientation. Otherwise, we want to turn the rover to
	// aim it at the target location.
	double targetAngle = drive_target(2);
	if (control_state == ControlState::FAR_FROM_TARGET_POSE)
	{
		double dx = drive_target(0) - pose(0);
		double dy = drive_target(1) - pose(1);
		targetAngle = atan2(dy, dx);
	}
	double relativeTargetAngle = getRelativeAngle(pose(2), targetAngle);
	thetaErr = relativeTargetAngle - pose(2);

	return KP_ANGLE * thetaErr;
}

pose_t Autonomous::choose_plan_target(const pose_t &pose)
{
  if (nav_state != NavState::GATE_TRAVERSE) {
    computeGateTargets(pose);
  }

  if (nav_state == NavState::GPS)
  {
    return getGPSTargetPose();
  }
  else if (nav_state == NavState::SEARCH_PATTERN)
  {
    return search_target;
  }
  else if (nav_state == NavState::POST_VISIBLE)
  {
    point_t post = (leftPostFilter.getSize() > 0) ? leftPostFilter.get() : rightPostFilter.get();
    double angle = std::atan2(post(1) - pose(1), post(0) - pose(0));
    // Offset the target by two meters to avoid crashing into the post
    pose_t plan_target({
        post(0) - 2.0 * cos(angle),
        post(1) - 2.0 * sin(angle),
        angle});
    return plan_target;
  }
  else if (nav_state == NavState::GATE_ALIGN)
  {
    return gate_targets.first;
  }
  else if (nav_state == NavState::GATE_TRAVERSE)
  {
    return gate_targets.second;
  }
  else // probably NavState::DONE
  {
    return {0,0,0};
  }
}

void Autonomous::updateLandmarkInformation(const transform_t &invTransform)
{
	// get landmark data and filter out invalid data points
	points_t landmarks = readLandmarks();
	printLandmarks(landmarks);
	if (urc_target.left_post_id < 0 || urc_target.left_post_id > landmarks.size())
	{
		log(LOG_ERROR, "Invalid left_post_id %d\n", urc_target.left_post_id);
		return;
	}
	point_t leftPostLandmark = landmarks[urc_target.left_post_id];
	point_t rightPostLandmark({0,0,0});
  if (urc_target.right_post_id != -1)
  {
    rightPostLandmark = landmarks[urc_target.right_post_id];
  }

  if (leftPostLandmark(2) != 0)
  {
    // transform and add the new data to the filter
    point_t leftLandmarkMapSpace = invTransform * leftPostLandmark;
    // the filtering is done on the target in map space to reduce any phase lag
    // caused by filtering
    leftPostFilter.get(leftLandmarkMapSpace);
  }
  if (rightPostLandmark(2) != 0)
  {
    point_t rightLandmarkMapSpace = invTransform * rightPostLandmark;
    rightPostFilter.get(rightLandmarkMapSpace);
  }

  const points_t landmarks_transformed = transformReadings(landmarks, VIZ_BASE_TF);
  publish_array(landmarks_transformed, landmarks_pub);
}

void Autonomous::computeGateTargets(const pose_t &pose)
{
  if (leftPostFilter.getSize() == 0 || rightPostFilter.getSize() == 0) return;

  point_t post_1 = leftPostFilter.get();
  point_t post_2 = rightPostFilter.get();
  point_t center = (post_1 + post_2) / 2;

  point_t offset{(post_1(1) - post_2(1)) / 2, (post_2(0) - post_1(0)) / 2, 0};
  pose_t goal_1 = center + 2 * offset;
  pose_t goal_2 = center - 2 * offset;

  // Choose the closest goal to be the first target
  if (dist(pose, goal_1, 1.0) < dist(pose, goal_2, 1.0))
  {
    gate_targets = std::make_pair(goal_1, goal_2);
  }
  else
  {
    gate_targets = std::make_pair(goal_2, goal_1);
  }

  // Angle between targets
  double angle = std::atan2(gate_targets.second(1) - gate_targets.first(1), gate_targets.second(0) - gate_targets.first(0));
  gate_targets.first(2) = angle;
  gate_targets.second(2) = angle;
}

void Autonomous::updateSearchTarget()
{
  // Replan to avoid waiting at current search point
  plan_cost = INFINITE_COST;
  // Set the search target to the next point in the search pattern
  search_target -= urc_target.approx_GPS;
  double radius = hypot(search_target(0), search_target(1));
  double scale = (radius + search_theta_increment) / radius;
  // Rotate the target counterclockwise by the theta increment
  search_target.topRows(2) =
    (Eigen::Matrix2d() << cos(search_theta_increment), -sin(search_theta_increment),
                sin(search_theta_increment), cos(search_theta_increment)).finished()
    * search_target.topRows(2) * scale;
  search_target += urc_target.approx_GPS;
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

void Autonomous::autonomyIter()
{
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

	// get the latest pose estimation
	poseEstimator.correct(gps);
	pose_t pose = poseEstimator.getPose();
	transform_t invTransform = toTransform(pose).inverse();

	if (nav_state == NavState::DONE)
	{
		leftPostFilter.reset(); // clear the cached data points
		rightPostFilter.reset();
		if (Globals::AUTONOMOUS)
		{
			setCmdVel(0, 0);
		}
    return;
	}

  updateLandmarkInformation(invTransform);
  pose_t plan_target = choose_plan_target(pose);
  update_nav_state(pose, plan_target);

  points_t lidar_scan = readLidarScan();
  // Our lidar sensor cannot see behind the rover. Until our mapping is improved
  // to remember obstacles behind the rover, we just assume that the way behind us
  // is blocked.
  double trail_dist = 3.0;
  for (double y = -1.8 * trail_dist; y < 1.8 * trail_dist; y += 0.5) {
    lidar_scan.push_back({-trail_dist, y, 1.0});
  }

  if (Globals::AUTONOMOUS) {
    if (plan_cost >= INFINITE_COST || ++time_since_plan % REPLAN_PERIOD == 0) {
      // TODO planning should happen in a separate thread
      // because it takes a long time
      point_t point_t_goal;
      point_t_goal.topRows(2) = plan_target.topRows(2);
      point_t_goal(2) = 1.0;
      point_t_goal = toTransform(pose) * point_t_goal;
      plan_t new_plan = getPlan(lidar_scan, point_t_goal, PLANNING_GOAL_REGION_RADIUS);
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
  }

  // Send lidar points to visualization
  const points_t lidar_scan_transformed = transformReadings(lidar_scan, VIZ_BASE_TF);
  log(LOG_DEBUG, "Publishing %d lidar points\n", lidar_scan_transformed.size());
  publish_array(lidar_scan_transformed, lidar_pub);

  // Send current pose and plan target to visualization
  const pose_t curr_pose_transformed = poseToDraw(pose, pose);
  publish(curr_pose_transformed, curr_pose_pub);
  const pose_t plan_target_transformed = poseToDraw(plan_target, pose);
  publish(plan_target_transformed, plan_target_pub);

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
    publish(start_plan_pose_transformed, plan_pub);

    bool found_target = false;
    double accumulated_cost = 0;
    for (int i = 0; i < plan_size; i++) {
      action_t action = plan.row(i);
      plan_pose(2) += action(0);
      plan_pose(0) += action(1) * cos(plan_pose(2));
      plan_pose(1) += action(1) * sin(plan_pose(2));
      transform_t tf_plan_pose = toTransform(plan_pose) * invTransform;
      if (i >= plan_idx && collides(tf_plan_pose, lidar_scan, 1.3)) { // stay 1.3 meters away
        // We'll replan next timestep
        // TODO strictly speaking, we don't need to replan if the collision happened
        // on a part of the plan that we've already passed. But I'm not quite sure how
        // to algorithmically decide which parts of the plan we've already passed.
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
        publish(curr_plan_transformed, plan_pub);
      }
    }
    if (!found_target) {
      plan_idx = plan_size - 1;
      driveTarget = plan_pose;
    }

    // This deals with drift
    // TODO maybe we should just replan if the distance drifts too far
    accumulated_cost += 2*dist(plan_pose, plan_target, 1.0);
    plan_cost = accumulated_cost;
  }

  // Send drive target to visualization
  const pose_t drive_target_transformed = poseToDraw(driveTarget, pose);
  publish(drive_target_transformed, drive_target_pub);

  // Send message to visualization that all data has been sent
  publish({NAN, NAN, NAN}, plan_pub);

  double d = dist(driveTarget, pose, 0);
  // There's an overlap where either state might apply, to prevent rapidly switching
  // back and forth between these two states.
  if (d < 0.2 && control_state == ControlState::FAR_FROM_TARGET_POSE)
  {
    control_state = ControlState::NEAR_TARGET_POSE;
  }
  if (d > 0.5 && control_state == ControlState::NEAR_TARGET_POSE)
  {
    control_state = ControlState::FAR_FROM_TARGET_POSE;
  }
  double thetaErr;
  double thetaVel = getThetaVel(driveTarget, pose, thetaErr);
  double driveSpeed = getLinearVel(driveTarget, pose, thetaErr);

  if (!Globals::E_STOP && Globals::AUTONOMOUS)
  {
    setCmdVel(thetaVel, driveSpeed);
    poseEstimator.predict(thetaVel, driveSpeed);
  }
}

pose_t Autonomous::getGPSTargetPose() const
{
	pose_t ret{urc_target.approx_GPS(0), urc_target.approx_GPS(1), 0.0};
	return ret;
}

static void printLandmarks(points_t& landmarks, int log_level){
	std::ostringstream stream;
	stream << "Landmarks: [";
	for(int i = 0; i < landmarks.size(); i++){
		stream << i << ": {"
			   << landmarks[i][0] << ", "
			   << landmarks[i][1] << ", "
			   << landmarks[i][2]
			   << "}";
		if (i < landmarks.size() - 1)
		{
			stream << ", ";
		}
	}
	stream << "}" << std::endl;
	log(log_level, stream.str().c_str());
}
