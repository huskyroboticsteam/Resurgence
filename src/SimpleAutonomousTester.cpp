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
    fm->addObstacle(PointXY{-5, 5}, PointXY{2, 5});
    fm->addObstacle(PointXY{-9, 7}, PointXY{-5, 6});
    fm->addObstacle(PointXY{2, 2}, PointXY{2, -2});
    autonomous.setWorldData(fm);
    REQUIRE(fm->lidarSees() == false);
    std::future<void> future = std::async(std::launch::async, [&]() {
        fm->callAutonomous();
    });
    std::future_status status = future.wait_for(std::chrono::seconds(10));
    REQUIRE(status == std::future_status::ready);
}