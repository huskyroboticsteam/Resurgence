#include "FakeMap.h"
#include "Autonomous.h"
#include "WorldData.h"
#include <catch2/catch.hpp>

TEST_CASE("full autonomous", "[autonomous]")
{
    PointXY p;
    p.x = -10;
    p.y = 10;
    Autonomous autonomous(p);
    auto fm = std::make_shared<FakeMap>(autonomous);
    autonomous.setWorldData(fm);
    REQUIRE(fm->lidarSees() == false);
    fm->callAutonomous();
}
