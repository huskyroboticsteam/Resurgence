#include <catch2/catch.hpp>
#include "Pathfinding/ObstacleMap.h"
#include <vector>
#include "Pathfinding/Point.h"

#define CATCH_CONFIG_MAIN

namespace Pathfinding
{
    //copied over from ObstacleMap
    int transform(int val, bool direction)
    {
        if(direction)
        {
            return val + (int)(1 - (val % 1));
        }
        return val - (int)(val % 1);
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
        for (Point p : vecOfPoints)
        {
            int x = p.x;
            int y = p.y;
            sol[transform(y, true)][transform(x, true)] = true;
            sol[transform(y, true)][transform(x, false)] = true;
            sol[transform(y, false)][transform(x, true)] = true;
            sol[transform(y, false)][transform(x, false)] = true;
        }
    }

    bool boolMapsEquals(bool obstacle_map[21][21], bool sol[21][21])
    {
        for (int i = 0; i < 21; i++)
        {
            for (int j = 0; j < 21; j++)
            {
                bool objMapVal = obstacle_map[i + 10][j+ 10];
                bool solVal = obstacle_map[i + 10][j + 10];
                if (objMapVal != solVal)
                {
                    return false;
                }
            }
        }
        return true;
    }
}

TEST_CASE("Create and fill ObstacleMap")
{
    std::vector<Point> vectorOfPoints = {
        Point { 2.0f, 2.0f },
        Point { -2.0f, -2.0f },
        Point { -3.0f, 3.0f },
        Point { 3.0f, -3.0f }
    };


    ObstacleMap objMap;
    objMap.update(vectorOfPoints);
    bool sol[21][21];
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    REQUIRE(Pathfinding::boolMapsEquals(objMap.obstacle_map, sol));
}