#include <catch2/catch.hpp>
#include <vector>
#include <iostream>

#include "../../src/Pathfinding/ObstacleMap.h"
#include "../../src/Pathfinding/Point.h"

#define CATCH_CONFIG_MAIN

namespace Pathfinding
{
    const int radius = 10;
    const int size = 2 * radius + 1;


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


    inline void fillFalse(bool sol[][size])
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                sol[i][j] = false;
            }
        }
    }

    bool withinBounds(int coord)
    {
        return coord < 21 && coord >= 0;
    }


    void getMapObjSol(bool sol[][size], std::vector<Point>& vectorOfPoints)
    {
        fillFalse(sol);
        for (Point p : vectorOfPoints)
        {
            int x = static_cast<int>(p.x) + radius;
            int y = static_cast<int>(p.y) + radius;
            if(withinBounds(x) && withinBounds(y)) {
                if (y + 1 < size && x + 1 < size) {
                    sol[y][x] = true;
                }
                if (y + 1 < size && x >= 0) {
                    sol[y][x + 1] = true;
                }
                if (y >= 0 && x + 1 < size) {
                    sol[y + 1][x] = true;
                }
                if (y >= 0 && x >= 0) {
                    sol[y + 1][x + 1] = true;
                }
            }
        }
    }

    bool boolMapsEquals(ObstacleMap& obsMap, bool sol[][size])
    {
        if(obsMap.size != size || obsMap.radius != radius) {
            return false;
        }
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                bool obsMapVal = obsMap.obstacle_map[i][j];
                bool solVal = sol[i][j];
                if (obsMapVal != solVal)
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
    //this test tests basic creation of ObstacleMap
    //also tests updating ObstacleMap as input updates
    //assumes center at 0, 0

    std::vector<Point> vectorOfPoints = {
        Point{2.0f, 2.0f},
        Point{-2.0f, -2.0f},
        Point{-3.0f, 3.0f},
        Point{3.0f, -3.0f},
        Point{5.0f, -5.0f},
        Point{1.0f, 1.0f}
    };

    ObstacleMap obsMap;
    bool sol[Pathfinding::size][Pathfinding::size];

    // Create an ObstacleMap out of given points and dummy double array (sol)
    // check that they have equal resulting structure by value
    obsMap.update(vectorOfPoints);
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    REQUIRE(Pathfinding::boolMapsEquals(obsMap, sol));

    // Adds point to the vector of points and update both obsMap and sol
    // check that they have equal resulting structure by value
    vectorOfPoints.push_back(Point{-7.0, 3.0});
    obsMap.update(vectorOfPoints);
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    REQUIRE(Pathfinding::boolMapsEquals(obsMap, sol));

}

TEST_CASE("Pathfinding OUT of BOUNDS")
{
    //this test tests with points outside of the bounds of ObstacleMap
    //assumes center at 0, 0

    std::vector<Point> vectorOfPoints = {
        Point{20.0f, 2.0f},
        Point{-2.0f, -2.0f},
        Point{-30.0f, 3.0f},
        Point{3.0f, -3.00008f},
        Point{5.0f, -5.770f},
        Point{1.50f, 1.60f}
    };

    ObstacleMap obsMap;
    bool sol[Pathfinding::size][Pathfinding::size];

    // Create an ObstacleMap out of given points and dummy double array (sol)
    // check that they have equal resulting structure by value
    obsMap.update(vectorOfPoints);
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    REQUIRE(Pathfinding::boolMapsEquals(obsMap, sol));
}