#include "../../src/filters/RollingAvgFilter.h"

#include <Eigen/Core>
#include <catch2/catch.hpp>

using namespace Catch::literals;
using filters::RollingAvgFilter;

namespace
{
Eigen::Matrix<double, 1, 1> create1D(double val)
{
	Eigen::Matrix<double, 1, 1> vec;
	vec << val;
	return vec;
}

Eigen::Matrix<double, 3, 1> create3D(double x1, double x2, double x3)
{
	Eigen::Matrix<double, 3, 1> vec;
	vec << x1, x2, x3;
	return vec;
}
} // namespace

TEST_CASE("RollingAvgFilter Test 1D")
{
	RollingAvgFilter<Eigen::Matrix<double, 1, 1>> filter(2);

	filter.get(create1D(1));
	filter.get(create1D(9));
	REQUIRE(filter.get()(0) == 5_a);

	filter.get(create1D(3));
	REQUIRE(filter.get()(0) == 6_a);

	filter.get(create1D(-3));
	REQUIRE(filter.get()(0) == Approx(0).margin(1e-9));
}

TEST_CASE("RollingAvgFilter Test 3D")
{
	RollingAvgFilter<Eigen::Vector3d> filter(3);

	filter.get(create3D(1, 1, 1));
	filter.get(create3D(2, 2, 2));
	filter.get(create3D(3, 3, 3));
	REQUIRE((filter.get() - create3D(2, 2, 2)).norm() == Approx(0).margin(1e-9));

	filter.get(create3D(4, 4, 4));
	REQUIRE((filter.get() - create3D(3, 3, 3)).norm() == Approx(0).margin(1e-9));
}
