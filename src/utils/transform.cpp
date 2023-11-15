#include "transform.h"

#include <Eigen/LU>

using namespace navtypes;

namespace util {

double quatToHeading(double qw, double qx, double qy, double qz) {
	Eigen::Quaterniond quat(qw, qx, qy, qz);
	return quatToHeading(quat);
}

double quatToHeading(const Eigen::Quaterniond& quat) {
	Eigen::Matrix3d rotMat = quat.normalized().toRotationMatrix();
	Eigen::Vector3d transformedX = rotMat * Eigen::Vector3d::UnitX();
	// flatten to xy-plane
	transformedX(2) = 0;
	// recover heading
	double heading = std::atan2(transformedX(1), transformedX(0));
	return heading;
}

Eigen::Quaterniond eulerAnglesToQuat(const navtypes::eulerangles_t& rpy) {
	Eigen::Quaterniond quat(Eigen::AngleAxisd(rpy.roll, Eigen::Vector3d::UnitX()) *
							Eigen::AngleAxisd(rpy.pitch, Eigen::Vector3d::UnitY()) *
							Eigen::AngleAxisd(rpy.yaw, Eigen::Vector3d::UnitZ()));
	return quat;
}

points_t transformReadings(const points_t& ps, const transform_t& tf) {
	transform_t tf_inv = tf.inverse();
	points_t readings({});
	for (point_t p : ps) {
		readings.push_back(tf_inv * p);
	}
	return readings;
}

trajectory_t transformTraj(const trajectory_t& traj, const transform_t& tf) {
	trajectory_t tf_traj({});
	for (const transform_t& tf_i : traj) {
		tf_traj.push_back(tf_i * tf);
	}
	return tf_traj;
}

bool collides(const transform_t& tf, const points_t& ps, double radius) {
	for (const point_t& p : ps) {
		if (p(2) == 0.0) {
			// This point is a "no data" point
			continue;
		}
		point_t tf_p = tf * p;
		tf_p(2) = 0;
		if (tf_p.norm() < radius)
			return true;
	}
	return false;
}

transform_t toTransformRotateFirst(double x, double y, double theta) {
	transform_t m;
	m << cos(theta), sin(theta), -x, -sin(theta), cos(theta), -y, 0, 0, 1;
	return m;
}

double closestHeading(double theta, double prev_theta) {
	while (theta < prev_theta - M_PI)
		theta += 2 * M_PI;
	while (theta > prev_theta + M_PI)
		theta -= 2 * M_PI;
	return theta;
}

pose_t toPose(const transform_t& trf, double prev_theta) {
	pose_t s = trf.inverse() * pose_t(0, 0, 1);
	double cos_theta = trf(0, 0);
	double sin_theta = -trf(1, 0);
	double theta = atan2(sin_theta, cos_theta);
	s(2) = closestHeading(theta, prev_theta);
	return s;
}

transform_t toTransform(const pose_t& pose) {
	return toTransformRotateFirst(0, 0, pose(2)) * toTransformRotateFirst(pose(0), pose(1), 0);
}

} // namespace util
