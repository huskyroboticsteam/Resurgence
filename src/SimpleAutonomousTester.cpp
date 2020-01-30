



#include "FakeMap.h"
#include "Autonomous.h"
#include "WorldData.h"
#include <catch2/catch.hpp>
TEST_CASE("robot starts going forwards", "[autonomous]") {
    PointXY p;
    p.x = 3.0;
    p.y = 2.0;
    Autonomous autonomous(p);
    auto fm = std::make_shared<FakeMap>(autonomous);
    autonomous.setWorldData(fm);
    REQUIRE(fm->getHeading() == Approx(0.0));
}

