#include "Util.h"
#include "navtypes.h"

#include <iostream>
#include <time.h>
#include <random>

#include <sys/time.h>

#include <Eigen/LU>

using namespace navtypes;

namespace util {
bool almostEqual(double a, double b, double threshold) {
	return std::abs(a - b) < threshold;
}

double quatToHeading(double qw, double qx, double qy, double qz) {
	Eigen::Quaterniond quat(qw, qx, qy, qz);
	return quatToHeading(quat);
}

double quatToHeading(Eigen::Quaterniond quat) {
	quat.normalize();
	Eigen::Matrix3d rotMat = quat.toRotationMatrix();
	Eigen::Vector3d transformedX = rotMat * Eigen::Vector3d::UnitX();
	// flatten to xy-plane
	transformedX(2) = 0;
	// recover heading
	double heading = std::atan2(transformedX(1), transformedX(0));
	return heading;
}

ScopedTimer::ScopedTimer(std::string name)
	: startTime(std::chrono::high_resolution_clock::now()), name(std::move(name)) {}

ScopedTimer::ScopedTimer() : ScopedTimer("") {}

ScopedTimer::~ScopedTimer() {
	if (!name.empty()) {
		auto now = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
		std::cout << "[" << name << "] ElapsedTime: " << elapsed.count() << "us\n";
	}
}

std::chrono::microseconds ScopedTimer::elapsedTime() const {
	auto now = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
	return elapsed;
}

points_t transformReadings(const points_t &ps, const transform_t &tf) {
  transform_t tf_inv = tf.inverse();
  points_t readings({});
  for (point_t p : ps) {
    readings.push_back(tf_inv * p);
  }
  return readings;
}

trajectory_t transformTraj(const trajectory_t &traj, const transform_t &tf) {
  trajectory_t tf_traj({});
  // TODO can probably simplify this method to use just one for loop
  for (const transform_t &tf_i : traj) {
    tf_traj.push_back(tf_i * tf);
  }
  return tf_traj;
}

bool collides(const transform_t &tf, const points_t &ps, double radius)
{
  for (const point_t &p : ps)
  {
    if (p(2) == 0.0) {
      // This point is a "no data" point
      continue;
    }
    point_t tf_p = tf * p;
    tf_p(2) = 0;
    if (tf_p.norm() < radius) return true;
  }
  return false;
}

transform_t toTransformRotateFirst(double x, double y, double theta) {
  transform_t m;
  m << cos(theta),  sin(theta), -x,
       -sin(theta), cos(theta), -y,
                0,           0,  1;
  return m;
}

double nearestHeadingBranch(double theta, double prev_theta) {
  while (theta < prev_theta - M_PI)
    theta += 2*M_PI;
  while (theta > prev_theta + M_PI)
    theta -= 2*M_PI;
  return theta;
}

pose_t toPose(const transform_t &trf, double prev_theta) {
  pose_t s = trf.inverse() * pose_t(0, 0, 1);
  double cos_theta = trf(0,0);
  double sin_theta = -trf(1,0);
  double theta = atan2(sin_theta, cos_theta);
  s(2) = nearestHeadingBranch(theta, prev_theta);
  return s;
}

transform_t toTransform(const pose_t &pose) {
  return toTransformRotateFirst(0, 0, pose(2)) * toTransformRotateFirst(pose(0), pose(1), 0);
}

/* Using two generators allows us to rerun random seeds for two-threaded programs.
 * There still might be some variation due to the dependence of the robot position
 * on when exactly each context switch occurs.
 *
 * For programs with more threads, a more sophisticated solution will be necessary.
 */
static std::normal_distribution<double> stdn_dist(0.0, 1.0);
static long seed = std::chrono::system_clock::now().time_since_epoch().count();
//long seed = 1626474823108702150;
static std::default_random_engine main_generator(seed);
static std::default_random_engine spin_generator(seed);

long getNormalSeed() {
  return seed;
}

double stdn(int thread_id) {
  if (thread_id == 0) return stdn_dist(main_generator);
  else return stdn_dist(spin_generator);
}

} // namespace util
