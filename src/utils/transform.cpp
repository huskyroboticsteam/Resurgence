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

double closestHeading(double theta, double prev_theta) {
	while (theta < prev_theta - M_PI)
		theta += 2 * M_PI;
	while (theta > prev_theta + M_PI)
		theta -= 2 * M_PI;
	return theta;
}

pose_t toPose(const transform_t& trf, double prev_theta) {
	pose_t s = trf.rightCols<1>();
	double cos_theta = trf(0, 0);
	double sin_theta = trf(1, 0);
	double theta = atan2(sin_theta, cos_theta);
	s(2) = closestHeading(theta, prev_theta);
	return s;
}

transform_t toTransform(const pose_t& pose) {
	return toTransform(pose(0), pose(1), pose(2));
}

transform_t toTransform(double x, double y, double theta) {
	transform_t m;
	m << cos(theta), -sin(theta), x, sin(theta), cos(theta), y, 0, 0, 1;
	return m;
}

} // namespace util
