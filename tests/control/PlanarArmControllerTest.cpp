#include "../../src/control/PlanarArmController.h"
#include "../../src/Constants.h"

#include <catch2/catch.hpp>

using namespace Catch::literals;
using namespace control;
using namespace std::chrono_literals;

TEST_CASE("Test Planar Arm Controller", "[control][planararmcontroller]") {
	navtypes::Vectord<2> vec({0, 0});
	kinematics::PlanarArmKinematics<2> kin_obj(vec, vec, vec, 0.0, 0);
	PlanarArmController<2> foo({0, 0}, kin_obj, Constants::arm::SAFETY_FACTOR);
}

TEST_CASE("Test Planar Arm Safety Factor", "[control][planararmcontroller]") {
	navtypes::Vectord<2> segLens({2.0, 3.0});
	navtypes::Vectord<2> minAngles({-M_PI, -M_PI});
	navtypes::Vectord<2> maxAngles({M_PI, M_PI});
	kinematics::PlanarArmKinematics<2> kin_obj(segLens, minAngles, maxAngles, 0.0, 0);

	PlanarArmController<2> foo({0, 0}, kin_obj, Constants::arm::SAFETY_FACTOR);
	foo.set_setpoint({5.0, 0.0});
}