#include <catch2/catch.hpp>
#include <iostream>
#include <vector>
#include "../../src/Pathfinding/Point.h"
#include "../../src/Pathfinding/Pather.h"
#include "../../src/Pathfinding/ObstacleMap.h"

#define CATCH_CONFIG_MAIN

namespace Pathfinding
{
    bool nextPointEqual(Point given, Point expect) {
        return given.x == expect.x && given.y == expect.y;
    }
}

TEST_CASE("Pathfinding")
{
    std::vector<Point> obstaclePosition= {
        Point{1.0f, 5.0f},
        Point{2.0f, 5.0f},
        Point{3.0f, 6.0f},
        Point{4.0f, 5.0f},
        Point{5.0f, 4.0f},
        Point{6.0f, 4.0f},
        Point{7.0f, 4.0f},
        Point{6.0f, 3.0f}
    };
    //Construct an obstacle map
    ObstacleMap obj;
    obj.update(obstaclePosition);
    
    //Construct a searching path object
    Pather searchingPath;
    Point nextPoint = searchingPath.getPath(obj, Point{10.0f, 10.0f});

    Point expectPoint = {1.0f, 0.0};
    //Check if returned next point is as expected.
    REQUIRE(Pathfinding::nextPointEqual(nextPoint, expectPoint))
}