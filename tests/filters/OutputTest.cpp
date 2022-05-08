#include <catch2/catch.hpp>

#include <iostream>

#include "../../src/filters/MultiSensorEKF.h"

Eigen::VectorXd outputFunc(const Eigen::VectorXd& x, const Eigen::VectorXd& b) {
	return (2 * x + b).topRows(x.size()-1);
}

TEST_CASE("Test Output Jacobian") {
	// auto tmp = Eigen::VectorXd::Ones(2);
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
