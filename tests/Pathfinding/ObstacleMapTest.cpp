#include <catch2/catch.hpp>
#include <vector>
#include <iostream>

#include "../../src/Pathfinding/ObstacleMap.h"
#include "../../src/Pathfinding/Point.h"

#define CATCH_CONFIG_MAIN

namespace Pathfinding
{
    //copied over from ObstacleMap
    int transform(int val, bool direction)
    {
        return val + direction;
    }

    void getMapObjSol(bool sol[21][21], std::vector<Point> vecOfPoints)
    {
        for (int i = 0; i < 21; i++)
        {
            for (int j = 0; j < 21; j++)
            {
                sol[i][j] = false;
            }
        }
        std::cout << "looped falses" << std::endl;
        for (Point p : vecOfPoints)
        {
            int x = static_cast<int>(p.x);
            int y = static_cast<int>(p.y);
            // sol[transform(y, true)][transform(x, true)] = true;
            // sol[transform(y, true)][transform(x, false)] = true;
            // sol[transform(y, false)][transform(x, true)] = true;
            // sol[transform(y, false)][transform(x, false)] = true;
            std::cout << transform(y, true) + 11 << std::endl;
            std::cout << transform(y, false) + 11 << std::endl;
            std::cout << transform(x, true) + 11 << std::endl;
            std::cout << transform(x, false) + 11 << std::endl;
            std::cout << "done with a point" << std::endl;
        }
    }

    bool boolMapsEquals(bool obstacle_map[21][21], bool sol[21][21])
    {
        for (int i = 0; i < 21; i++)
        {
            for (int j = 0; j < 21; j++)
            {
                bool objMapVal = obstacle_map[i][j];
                bool solVal = sol[i][j];
                if (objMapVal != solVal)
                {
                    return false;
                }
            }
        }
        return true;
    }
}

TEST_CASE("Pathfinding")
{
    std::vector<Point> vectorOfPoints = {
        Point { 2.0f, 2.0f },
        Point { -2.0f, -2.0f },
        Point { -3.0f, 3.0f },
        Point { 3.0f, -3.0f }
    };


    ObstacleMap objMap;
    objMap.update(vectorOfPoints);
    objMap.print();
    bool sol[21][21];
    std::cout << "created sol arr" << std::endl;
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    REQUIRE(Pathfinding::boolMapsEquals(objMap.obstacle_map, sol));
}