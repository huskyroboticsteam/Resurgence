#include "../../src/control/PlanarArmController.h"

#include <catch2/catch.hpp>

using namespace Catch::literals;
using namespace control;
using namespace std::chrono_literals;

TEST_CASE("Test Planar Arm Controller", "[control][planararmcontroller]") {
    navtypes::Vectord<2> vec({0, 0});
    kinematics::PlanarArmKinematics<2> kin_obj(vec, vec, vec, 0.0, 0);
    PlanarArmController<2> foo({0,0}, kin_obj);
}