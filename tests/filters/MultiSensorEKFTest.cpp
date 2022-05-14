#include <stdexcept>

#include <catch2/catch.hpp>
#define eigen_assert(x)                                                                       \
	if (!(x)) {                                                                               \
		throw std::runtime_error(#x);                                                         \
	}

#include "../../src/filters/MultiSensorEKF.h"

#include <Eigen/Core>

using namespace filters;
using namespace filters::statespace;

TEST_CASE("Test Output Jacobian", "[filters]") {
	auto outputFunc = [](const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
		return (2 * x + b).topRows(x.size()-1);
	};

	Eigen::VectorXd std(Eigen::VectorXd::Ones(3));
	filters::statespace::NoiseCovMatX cov(std, 3, 2);
	filters::Output o(3, 2, 3, outputFunc, cov);

	Eigen::VectorXd state(Eigen::VectorXd::Ones(3));

	Eigen::MatrixXd jacobX = o.outputFuncJacobianX(state);
	Eigen::MatrixXd jacobV = o.outputFuncJacobianV(state);

	Eigen::MatrixXd jacobXTrue(2,3);
	jacobXTrue << 2, 0, 0,
				  0, 2, 0;

	REQUIRE((jacobX - jacobXTrue).array().abs().maxCoeff() <= 1e-4);
	REQUIRE((jacobV - jacobXTrue/2).array().abs().maxCoeff() <= 1e-4);
}

TEST_CASE("MultiSensorEKF Test Runtime Errors", "[filters]") {
	constexpr int stateDim = 3;
	constexpr int inputDim = 2;

	auto outputFunc1 = [](const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
		return (2 * x + b.topRows(stateDim)).topRows(x.size() - 1);
	};
	auto outputFunc2 = [](const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
		return Eigen::Vector3d::Ones() * ((2 * x + b).norm());
	};
	auto stateFunc = [](const Vectord<stateDim>& x, const Vectord<inputDim>& u,
						const Vectord<stateDim>& noise) { return x + noise; };

	NoiseCovMat<-1,-1,-1> covMat1(Eigen::MatrixXd::Identity(4, 4).eval(), stateDim, 2);
	Output out1(stateDim, 2, 4, outputFunc1, covMat1);
	NoiseCovMat<-1,-1,-1> covMat2(Eigen::MatrixXd::Identity(3, 3).eval(), stateDim, 3);
	Output out2(stateDim, 3, 3, outputFunc2, covMat2);
	NoiseCovMat<stateDim, stateDim, inputDim> processNoise(
		Eigen::Matrix<double, stateDim, stateDim>::Identity().eval());
	std::array<Output, 2> outputs = {out1, out2};
	filters::MultiSensorEKF<stateDim, inputDim, stateDim, 2> ekf(stateFunc, processNoise, 0.1,
																 outputs);
	REQUIRE_NOTHROW(ekf.predict(Eigen::Vector2d::Zero().eval()));
	REQUIRE_NOTHROW(ekf.correct<0>(Eigen::Vector2d::Ones().eval()));
	REQUIRE_NOTHROW(ekf.correct<1>(Eigen::Vector3d::Ones().eval() * 3));
}
