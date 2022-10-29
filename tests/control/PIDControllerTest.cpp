#include "../../src/control/PIDControl.h"

#include <catch2/catch.hpp>

using namespace Catch::literals;
using namespace control;
using namespace robot::types;
using namespace std::chrono_literals;

TEST_CASE("Test PID Controller", "[control][pid]") {
	constexpr int dim = 2;
	datatime_t startTime(0s);

	SECTION("Test Has Target") {
		std::array<PIDGains, dim> gains = {PIDGains{.kP = .2}, PIDGains{.kP = .3}};
		PIDController<dim> controller(gains);

		REQUIRE_FALSE(controller.hasTarget());
		REQUIRE(controller.getOutput(startTime, {0, 0}).isZero());

		controller.setTarget({1, 1});
		REQUIRE(controller.hasTarget());
		REQUIRE_FALSE(controller.getOutput(startTime, {0, 0}).isZero());

		controller.reset();
		REQUIRE_FALSE(controller.hasTarget());
		REQUIRE(controller.getOutput(startTime, {0, 0}).isZero());
	}

	SECTION("Test P control") {
		std::array<PIDGains, dim> gains = {PIDGains{.kP = .2}, PIDGains{.kP = .3}};
		PIDController<dim> controller(gains);

		controller.setTarget({1, 1});

		Eigen::Array2d output = controller.getOutput(startTime, {0, 0});
		REQUIRE(output[0] == 0.2_a);
		REQUIRE(output[1] == 0.3_a);
	}

	SECTION("Test D control") {
		std::array<PIDGains, dim> gains = {PIDGains{.kP = .2, .kD = 0.1},
										   PIDGains{.kP = .3, .kD = 0.2}};
		PIDController<dim> controller(gains);

		controller.setTarget({1, 1});

		REQUIRE(controller.getOutput(startTime, {0, 0}).isApprox(Eigen::Array2d{0.2, 0.3}));
		REQUIRE(controller.getOutput(startTime + 500ms, {0.5, 0.5})
					.isApprox(Eigen::Array2d{0.2, 0.35}));
	}

	SECTION("Test I control - With iZone") {
		std::array<PIDGains, dim> gains = {PIDGains{.kP = .2, .kI = 0.1, .iZone = 0.15},
										   PIDGains{.kP = .3, .kI = 0.2, .iZone = 0.15}};
		PIDController<dim> controller(gains);

		controller.setTarget({1, 1});

		// start up accumulator
		REQUIRE(controller.getOutput(startTime, {0, 0}).isApprox(Eigen::Array2d{0.2, 0.3}));
		// error not within izone, i term is zero
		REQUIRE(controller.getOutput(startTime + 500ms, {0.5, 0.5})
					.isApprox(Eigen::Array2d{0.1, 0.15}));
		
		controller.reset();
		controller.setTarget({1, 1});
		// start up accumulator
		REQUIRE(controller.getOutput(startTime, {0, 0}).isApprox(Eigen::Array2d{0.2, 0.3}));
		// error within izone, i term is nonzero
		REQUIRE(controller.getOutput(startTime + 500ms, {0.9, 0.9})
					.isApprox(Eigen::Array2d{0.0425, 0.075}));
	}

	SECTION("Test I control - Without iZone") {
		std::array<PIDGains, dim> gains = {PIDGains{.kP = .2, .kI = 0.1},
										   PIDGains{.kP = .3, .kI = 0.2}};
		PIDController<dim> controller(gains);

		controller.setTarget({1, 1});

		// start up accumulator
		REQUIRE(controller.getOutput(startTime, {0, 0}).isApprox(Eigen::Array2d{0.2, 0.3}));
		REQUIRE(controller.getOutput(startTime + 500ms, {0.5, 0.5})
					.isApprox(Eigen::Array2d{0.1125, 0.175}));
		
		controller.reset();
		controller.setTarget({1, 1});
		// start up accumulator
		REQUIRE(controller.getOutput(startTime, {0, 0}).isApprox(Eigen::Array2d{0.2, 0.3}));
		REQUIRE(controller.getOutput(startTime + 500ms, {0.9, 0.9})
					.isApprox(Eigen::Array2d{0.0425, 0.075}));
	}
}