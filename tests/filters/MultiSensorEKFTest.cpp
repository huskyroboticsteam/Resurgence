#include <catch2/catch.hpp>

#include <stdexcept>
#define eigen_assert(x) if (!(x)) { throw std::runtime_error(#x); }

#include "../../src/filters/MultiSensorEKF.h"

#include <Eigen/Core>

#include <iostream>

using namespace filters;
using namespace filters::statespace;

constexpr int stateDim = 3;
constexpr int inputDim = 2;

Eigen::VectorXd outputFunc1(const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
	return (2 * x + b.topRows(stateDim)).topRows(x.size()-1);
}

Eigen::VectorXd outputFunc2(const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
	return Eigen::Vector3d::Ones() * ((2 * x + b).norm());
}

Eigen::Matrix<double, stateDim, 1> stateFunc(const Vectord<stateDim>& x, const Vectord<inputDim>& u, const Vectord<stateDim>& noise) {
	return x + noise;
}

TEST_CASE("MultiSensorEKFTest") {
	NoiseCovMatX covMat1(Eigen::MatrixXd::Identity(4, 4).eval(), stateDim, 2);
	Output out1(stateDim, 2, 4, outputFunc1, covMat1);
	NoiseCovMatX covMat2(Eigen::MatrixXd::Identity(3, 3).eval(), stateDim, 3);
	Output out2(stateDim, 3, 3, outputFunc2, covMat2);
	NoiseCovMat<stateDim, stateDim, inputDim> processNoise(Eigen::Matrix<double, stateDim, stateDim>::Identity().eval());
	std::array<Output, 2> outputs = {out1, out2};
	filters::MultiSensorEKF<stateDim, inputDim, stateDim, 2> ekf(stateFunc, processNoise, 0.1, outputs);
	REQUIRE_NOTHROW(ekf.predict(Eigen::Vector2d::Zero().eval()));
	REQUIRE_NOTHROW(ekf.correct<0>(Eigen::Vector2d::Ones().eval()));
	ekf.correct<1>(Eigen::Vector3d::Ones().eval() * 3);
}
