#include "FakeMap.h"
#include "Autonomous.h"
#include "WorldData.h"
#include <catch2/catch.hpp>
#include <future>
#include <thread>

TEST_CASE("full autonomous", "[autonomous]")
{
    PointXY p;
    p.x = -10;
    p.y = 10;
    Autonomous autonomous(p);
    auto fm = std::make_shared<FakeMap>(autonomous);
    fm->addObstacle(PointXY{-5, 5}, PointXY{2, 5});
    fm->addObstacle(PointXY{-9, 7}, PointXY{-5, 6});
    fm->addObstacle(PointXY{2, 2}, PointXY{2, -2});
    autonomous.setWorldData(fm);
    REQUIRE(fm->lidarSees() == false);
    REQUIRE(fm->callAutonomous(200));
}
