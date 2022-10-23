#include <catch2/catch.hpp>

#include "../../src/control/JacobianController.h"
#include "../../src/control/JacobianVelController.h"

using namespace control;

TEST_CASE("JacobianController", "[control]") {
	Eigen::Matrix2d mat = Eigen::Matrix2d::Random();
	auto kinFn = [=](const Eigen::Vector2d& x) { return mat * x; };
	JacobianController<2,2> jc(kinFn, {}, {2, 2}, {1, 1});
	JacobianVelController<2,2> jvc(kinFn, {});
	// TODO: write tests
}
