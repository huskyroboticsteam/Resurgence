#include "FakeMap.h"
#include "Autonomous.h"
#include "WorldData.h"
#include "simulator/utils.h"
#include <catch2/catch.hpp>
#include <future>
#include <thread>

TEST_CASE("full autonomous", "[autonomous]")
{
    point_t gps_target {-10, 10, 1};
    URCLeg leg {0, 0, gps_target};
    Autonomous autonomous(leg);
    auto fm = std::make_shared<FakeMap>(autonomous);
    fm->addObstacle(PointXY{-5, 5}, PointXY{2, 5});
    fm->addObstacle(PointXY{-9, 7}, PointXY{-5, 6});
    fm->addObstacle(PointXY{2, 2}, PointXY{2, -2});
    autonomous.setWorldData(fm);
    REQUIRE(fm->callAutonomous(200));
    PointXY pos = fm->getRobotPos();
    REQUIRE(pos.x == Approx(-10).margin(0.1));
    REQUIRE(pos.y == Approx(10).margin(0.1));
}

TEST_CASE("target at start, no obstacles", "[autonomous]")
{
    point_t gps_target {0, 0, 1};
    URCLeg leg {0, 0, gps_target};
    Autonomous autonomous(leg);
    auto fm = std::make_shared<FakeMap>(autonomous);
    autonomous.setWorldData(fm);
    REQUIRE(fm->callAutonomous(200));
    PointXY pos = fm->getRobotPos();
    REQUIRE(pos.x == Approx(0).margin(0.1));
    REQUIRE(pos.y == Approx(0).margin(0.1));
}

TEST_CASE("robot boxed in, should timeout", "[autonomous]")
{
    point_t gps_target {5, 5, 1};
    URCLeg leg {0, 0, gps_target};
    Autonomous autonomous(leg);
    auto fm = std::make_shared<FakeMap>(autonomous);
    autonomous.setWorldData(fm);
    fm->addObstacle(PointXY{-2, 2}, PointXY{2, 2});
    fm->addObstacle(PointXY{-2, 2}, PointXY{-2, -2});
    fm->addObstacle(PointXY{-2, -2}, PointXY{2, -2});
    fm->addObstacle(PointXY{2, -2}, PointXY{2, 2});
    REQUIRE(!(fm->callAutonomous(50)));
    PointXY pos = fm->getRobotPos();
    REQUIRE(pos.x == Approx(0).margin(2));
    REQUIRE(pos.y == Approx(0).margin(2));
}

TEST_CASE("long line obstacle", "[autonomous]")
{
    point_t gps_target {0, 10, 1};
    URCLeg leg {0, 0, gps_target};
    Autonomous autonomous(leg);
    auto fm = std::make_shared<FakeMap>(autonomous);
    autonomous.setWorldData(fm);
    fm->addObstacle(PointXY{-10, 3}, PointXY{10, 3});
    REQUIRE((fm->callAutonomous(200)));
    PointXY pos = fm->getRobotPos();
    REQUIRE(pos.x == Approx(0).margin(0.1));
    REQUIRE(pos.y == Approx(10).margin(0.1));
}
