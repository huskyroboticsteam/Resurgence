#include <catch2/catch.hpp>
#include <vector>
#include <iostream>

#include "../../src/Pathfinding/ObstacleMap.h"
#include "../../src/Pathfinding/Point.h"

#define CATCH_CONFIG_MAIN

namespace Pathfinding
{
    void print(bool sol[21][21])
    {
        for (int i = 0; i < 21; i++)
        {
            for (int j = 0; j < 21; j++)
            {
                if(sol[i][j])
                {
                    std::cout << 1 << " ";
                } else {
                    std::cout << 0 << " ";
                }
            }
            std::cout << std::endl;
        }
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
            std::cout << p.y << " cast " << static_cast<int>(p.y) << std::endl;
            int x = static_cast<int>(p.x);
            int y = static_cast<int>(p.y);
            assert(y + 11 < 21 && y + 11 >= 0);
            sol[y + 1 + 10][x + 1 + 10] = true;
            sol[y + 1 + 10][x + 10] = true;
            sol[y + 10][x + 1 + 10] = true;
            sol[y + 10][x + 10] = true;
            std::cout << "done with a point" << std::endl;
        }
        std::cout << sol << std::endl;
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
        Point{2.0f, 2.0f},
        Point{-2.0f, -2.0f},
        Point{-3.0f, 3.0f},
        Point{3.0f, -3.0f},
        Point{5.0f, -5.0f}
    };

    // assert(vectorOfPoints[4].y == -5.0f);
    ObstacleMap objMap;
    objMap.update(vectorOfPoints);
    objMap.print();
    // assert(vectorOfPoints[4].y == -5.0f);
    std::cout << vectorOfPoints.back().y << std::endl;

    bool sol[21][21];
    std::cout << "created sol arr" << std::endl;
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    Pathfinding::print(sol);
    REQUIRE(Pathfinding::boolMapsEquals(objMap.obstacle_map, sol));
}