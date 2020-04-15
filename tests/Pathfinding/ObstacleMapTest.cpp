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

    void getMapObjSol(bool sol[21][21], std::vector<Point>& vecOfPoints)
    {
        fillFalse(sol);
        for (Point p : vecOfPoints)
        {
            int x = static_cast<int>(p.x);
            int y = static_cast<int>(p.y);
            assert(y + 11 < 21 && y + 11 >= 0);
            sol[y + 1 + 10][x + 1 + 10] = true;
            sol[y + 1 + 10][x + 10] = true;
            sol[y + 10][x + 1 + 10] = true;
            sol[y + 10][x + 10] = true;
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

    void fillFalse(bool sol[21][21])
    {
        for (int i = 0; i < 21; i++)
        {
            for (int j = 0; j < 21; j++)
            {
                sol[i][j] = false;
            }
        }
    }
}

TEST_CASE("Pathfinding")
{
    std::vector<Point> vectorOfPoints = {
        Point{2.0f, 2.0f},
        Point{-2.0f, -2.0f},
        Point{-3.0f, 3.0f},
        Point{3.0f, -3.0f},
        Point{5.0f, -5.0f},
        Point{1.0f, 1.0f}
    };

    ObstacleMap objMap;
    std::cout << "before update " << vectorOfPoints.back().y << std::endl;

    objMap.update(vectorOfPoints);
    std::cout << "ObstacleMap" << std::endl;
    objMap.print();

    bool sol[21][21];
    // std::cout << "before sol " << vectorOfPoints.back().y << std::endl;
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    std::cout << "after sol " << vectorOfPoints.back().y << std::endl;
    std::cout << "sol" << std::endl;

    Pathfinding::print(sol);
    REQUIRE(Pathfinding::boolMapsEquals(objMap.obstacle_map, sol));
}