#include "FakeMap.h"
#include "Autonomous.h"
#include "WorldData.h"
#include <catch2/catch.hpp>
#include <future>
#include <thread>

constexpr double CONTROL_HZ = 10.0;

TEST_CASE("full autonomous", "[autonomous]")
{
    PointXY p;
    p.x = -10;
    p.y = 10;
    Autonomous autonomous(p, CONTROL_HZ);
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
    PointXY p;
    p.x = 0;
    p.y = 0;
    Autonomous autonomous(p, CONTROL_HZ);
    auto fm = std::make_shared<FakeMap>(autonomous);
    autonomous.setWorldData(fm);
    REQUIRE(fm->callAutonomous(200));
    PointXY pos = fm->getRobotPos();
    REQUIRE(pos.x == Approx(0).margin(0.1));
    REQUIRE(pos.y == Approx(0).margin(0.1));
}

TEST_CASE("robot boxed in, should timeout", "[autonomous]")
{
    PointXY p;
    p.x = 5;
    p.y = 5;
    Autonomous autonomous(p);
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
    PointXY p;
    p.x = 0;
    p.y = 10;
    Autonomous autonomous(p);
    auto fm = std::make_shared<FakeMap>(autonomous);
    autonomous.setWorldData(fm);
    fm->addObstacle(PointXY{-10, 3}, PointXY{10, 3});
    REQUIRE((fm->callAutonomous(200)));
    PointXY pos = fm->getRobotPos();
    REQUIRE(pos.x == Approx(0).margin(0.1));
    REQUIRE(pos.y == Approx(10).margin(0.1));
}