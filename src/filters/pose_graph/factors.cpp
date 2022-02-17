#include "factors.h"

#include "graph.h"

namespace filters::pose_graph {

template <int D> using jacobian = Eigen::Matrix<double, Eigen::Dynamic, D>;

template <int D> using measurement = Eigen::Matrix<double, D, 1>;

template <int D> using covariance = Eigen::Matrix<double, D, D>;

OdomFactor::OdomFactor(int idx1, int idx2, double sigma, double m)
	: Factor<1>(covariance<1>{1 / sigma / sigma}, measurement<1>{m}), _idx1(idx1),
	  _idx2(idx2) {}

measurement<1> OdomFactor::f(const values& x) {
	return measurement<1>{x(_idx2) - x(_idx1)};
}

jacobian<1> OdomFactor::jf(const values& x) {
	jacobian<1> j = jacobian<1>::Zero(x.size(), 1);
	j(_idx2) = 1;
	j(_idx1) = -1;
	return j;
}

bool OdomFactor::shiftIndices(int poseSize, int firstPoseIdx) {
	_idx1 -= poseSize;
	_idx2 -= poseSize;
	return (_idx1 >= firstPoseIdx && _idx2 >= firstPoseIdx);
}

GPSFactor::GPSFactor(int idx, double sigma, double m)
	: Factor<1>(covariance<1>{1 / sigma / sigma}, measurement<1>{m}), _idx(idx) {}

measurement<1> GPSFactor::f(const values& x) {
	return measurement<1>{x(_idx)};
}

jacobian<1> GPSFactor::jf(const values& x) {
	jacobian<1> j = jacobian<1>::Zero(x.size(), 1);
	j(_idx) = 1;
	return j;
}

bool GPSFactor::shiftIndices(int poseSize, int firstPoseIdx) {
	_idx -= poseSize;
	return _idx >= firstPoseIdx;
}

LandmarkFactor2D::LandmarkFactor2D(int lmPose, int sensorPose, const covariance<2>& sigma_inv,
								   const measurement<2> m)
	: Factor<2>(sigma_inv, m), _lmPose(lmPose), _sensorPose(sensorPose) {}

measurement<2> LandmarkFactor2D::f(const values& x) {
	double px(0), py(0), theta(0);
	if (_sensorPose >= 0) {
		px = x(_sensorPose);
		py = x(_sensorPose + 1);
		theta = x(_sensorPose + 2);
	}
	return measurement<2>{(x(_lmPose) - px) * cos(theta) + (x(_lmPose + 1) - py) * sin(theta),
						  (x(_lmPose) - px) * (-sin(theta)) +
							  (x(_lmPose + 1) - py) * cos(theta)};
}

jacobian<2> LandmarkFactor2D::jf(const values& x) {
	jacobian<2> j = jacobian<2>::Zero(x.size(), 2);
	double px(0), py(0), theta(0);
	if (_sensorPose >= 0) {
		px = x(_sensorPose);
		py = x(_sensorPose + 1);
		theta = x(_sensorPose + 2);
	}
	j(_lmPose + 0, 0) = cos(theta);
	j(_lmPose + 1, 0) = sin(theta);
	j(_lmPose + 0, 1) = -sin(theta);
	j(_lmPose + 1, 1) = cos(theta);
	if (_sensorPose >= 0) {
		j(_sensorPose + 0, 0) = -cos(theta);
		j(_sensorPose + 1, 0) = -sin(theta);
		j(_sensorPose + 2, 0) =
			(x(_lmPose) - px) * (-sin(theta)) + (x(_lmPose + 1) - py) * cos(theta);
		j(_sensorPose + 0, 1) = sin(theta);
		j(_sensorPose + 1, 1) = -cos(theta);
		j(_sensorPose + 2, 1) =
			(x(_lmPose) - px) * (-cos(theta)) + (x(_lmPose + 1) - py) * (-sin(theta));
	}
	return j;
}

bool LandmarkFactor2D::shiftIndices(int poseSize, int firstPoseIdx) {
	if (_sensorPose >= 0) {
		_sensorPose -= poseSize;
		return _sensorPose >= firstPoseIdx;
	}
	return true;
}

// This is identical to LandmarkFactor2D except for one extra measurement (an angle
// difference).
// TODO figure out how to remove all the duplicate code.
OdomFactor2D::OdomFactor2D(int pose2, int pose1, const covariance<3>& sigma_inv,
						   const measurement<3> m)
	: Factor<3>(sigma_inv, m), _pose2(pose2), _pose1(pose1) {}

measurement<3> OdomFactor2D::f(const values& x) {
	double px(0), py(0), theta(0);
	if (_pose1 >= 0) {
		px = x(_pose1);
		py = x(_pose1 + 1);
		theta = x(_pose1 + 2);
	}
	return measurement<3>{(x(_pose2) - px) * cos(theta) + (x(_pose2 + 1) - py) * sin(theta),
						  (x(_pose2) - px) * (-sin(theta)) + (x(_pose2 + 1) - py) * cos(theta),
						  x(_pose2 + 2) - theta};
}

jacobian<3> OdomFactor2D::jf(const values& x) {
	jacobian<3> j = jacobian<3>::Zero(x.size(), 3);
	double px(0), py(0), theta(0);
	if (_pose1 >= 0) {
		px = x(_pose1);
		py = x(_pose1 + 1);
		theta = x(_pose1 + 2);
	}
	j(_pose2 + 0, 0) = cos(theta);
	j(_pose2 + 1, 0) = sin(theta);
	j(_pose2 + 0, 1) = -sin(theta);
	j(_pose2 + 1, 1) = cos(theta);
	j(_pose2 + 2, 2) = 1;
	if (_pose1 >= 0) {
		j(_pose1 + 0, 0) = -cos(theta);
		j(_pose1 + 1, 0) = -sin(theta);
		j(_pose1 + 2, 0) =
			(x(_pose2) - px) * (-sin(theta)) + (x(_pose2 + 1) - py) * cos(theta);
		j(_pose1 + 0, 1) = sin(theta);
		j(_pose1 + 1, 1) = -cos(theta);
		j(_pose1 + 2, 1) =
			(x(_pose2) - px) * (-cos(theta)) + (x(_pose2 + 1) - py) * (-sin(theta));
		j(_pose1 + 2, 2) = -1;
	}
	return j;
}

bool OdomFactor2D::shiftIndices(int poseSize, int firstPoseIdx) {
	_pose2 -= poseSize;
	if (_pose1 >= 0) {
		_pose1 -= poseSize;
		return _pose1 >= firstPoseIdx && _pose2 >= firstPoseIdx;
	} else {
		return _pose2 >= firstPoseIdx;
	}
}

} // namespace filters::pose_graph
