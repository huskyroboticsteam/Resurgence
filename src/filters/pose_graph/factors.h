#pragma once

#include "graph.h"

namespace filters::pose_graph {

template <int D> using jacobian = Eigen::Matrix<double, Eigen::Dynamic, D>;

template <int D> using measurement = Eigen::Matrix<double, D, 1>;

template <int D> using covariance = Eigen::Matrix<double, D, D>;

template <int D> class Factor : public AbstractFactor {
public:
	const covariance<D> _sigma_inv;
	const measurement<D> _measurement;

	Factor(const covariance<D>& sigma_inv, const measurement<D>& measurement)
		: _sigma_inv(sigma_inv), _measurement(measurement) {}

	virtual measurement<D> f(const values& /*x*/) = 0;
	virtual jacobian<D> jf(const values& /*x*/) = 0;

	virtual double eval(const values& x) {
		measurement<D> diff = f(x) - _measurement;
		return 0.5 * (diff.transpose() * _sigma_inv * diff)(0, 0);
	}

	virtual values gradient_at(const values& x) {
		return jf(x) * (_sigma_inv * (f(x) - _measurement));
	}

	virtual hessian hessian_at(const values& x) {
		jacobian<D> j = jf(x);
		return j * _sigma_inv * j.transpose();
	}
};

class OdomFactor : public Factor<1> {
	int _idx1, _idx2;

public:
	OdomFactor(int idx1, int idx2, double sigma, double m);
	virtual measurement<1> f(const values& x);
	virtual jacobian<1> jf(const values& x);
	virtual bool shiftIndices(int poseSize, int firstPoseIdx);
};

class GPSFactor : public Factor<1> {
	int _idx;

public:
	GPSFactor(int idx, double sigma, double m);
	virtual measurement<1> f(const values& x);
	virtual jacobian<1> jf(const values& x);
	virtual bool shiftIndices(int poseSize, int firstPoseIdx);
};

class LandmarkFactor2D : public Factor<2> {
	int _lmPose, _sensorPose;

public:
	LandmarkFactor2D(int lmPose, int sensorPose, const covariance<2>& sigma_inv,
					 const measurement<2> m);
	virtual measurement<2> f(const values& x);
	virtual jacobian<2> jf(const values& x);
	virtual bool shiftIndices(int poseSize, int firstPoseIdx);
};

class OdomFactor2D : public Factor<3> {
	int _pose2, _pose1;

public:
	OdomFactor2D(int pose2, int pose1, const covariance<3>& sigma_inv, const measurement<3> m);
	virtual measurement<3> f(const values& x);
	virtual jacobian<3> jf(const values& x);
	virtual bool shiftIndices(int poseSize, int firstPoseIdx);
};

} // namespace filters::pose_graph
