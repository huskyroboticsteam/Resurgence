#include <catch2/catch.hpp>
#include <vector>
#include <iostream>

#include "../../src/Pathfinding/ObstacleMap.h"
#include "../../src/Pathfinding/Point.h"

#define CATCH_CONFIG_MAIN

namespace Pathfinding
{
    //constant constraints for dummy double array sol
    const int radius = 10;
    const int size = 2 * radius + 1;

    //prints visualization of bool double array where 1 is True and 0 is false
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

    //sets all valus of sol to false
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

    //checks if coord is within bounds of the dummy double array dimensions
    bool withinBounds(int coord)
    {
        return coord < size && coord >= 0;
    }

    //makes dummy obstacle map resultant array out of sol
    //similar to ObstacleMap.update(<vectorOfPoints>)
    void getMapObjSol(bool sol[][size], std::vector<Point>& vectorOfPoints)
    {
        fillFalse(sol);
        for (Point p : vectorOfPoints)
        {
            int x = static_cast<int>(p.x) + radius;
            int y = static_cast<int>(p.y) + radius;
            if(withinBounds(x) && withinBounds(y)) {
                if (y + 1 < size && x + 1 < size) {
                    sol[y + 1][x + 1] = true;
                }
                if (y + 1 < size && x >= 0) {
                    sol[y + 1][x] = true;
                }
                if (y >= 0 && x + 1 < size) {
                    sol[y][x + 1] = true;
                }
                if (y >= 0 && x >= 0) {
                    sol[y][x] = true;
                }
            }
        }
    }

    //checks equivalancy of size and values of obstacleMap object and sol 
    bool boolMapsEquals(bool obstacleMap[][size], bool sol[][size])
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                bool obsMapVal = obstacleMap[i][j];
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

TEST_CASE("ObstacleMap")
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
    REQUIRE(Pathfinding::boolMapsEquals(obsMap.obstacle_map, sol));

    // Adds point to the vector of points and update both obsMap and sol
    // check that they have equal resulting structure by value
    vectorOfPoints.push_back(Point{-7.0, 3.0});
    obsMap.update(vectorOfPoints);
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    REQUIRE(Pathfinding::boolMapsEquals(obsMap.obstacle_map, sol));

}

TEST_CASE("ObstacleMap OUT OF BOUNDS")
{
    //this test tests with points outside of the bounds of ObstacleMap
    //and points with non-rounded float values
    //assumes center at 0, 0

    std::vector<Point> vectorOfPoints = {
        Point{20.0f, 2.0f}, //Out OF BOUNDS
        Point{-2.0f, -2.0f},
        Point{-30.0f, 3.0f}, //Out OF BOUNDS
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
    REQUIRE(Pathfinding::boolMapsEquals(obsMap.obstacle_map, sol));
}

TEST_CASE("ObstacleMap CHANGE CENTER")
{
    std::vector<Point> vectorOfPoints = {
        Point{10.0f, 10.0f},
        Point{10.0f, -10.0f},
        Point{-10.0f, 10.0f},
        Point{-10.0f, -10.0f}
    };

    ObstacleMap obsMap;
    std::cout << "rx: " << obsMap.robotX << ", ry: " << obsMap.robotY << std::endl;
    bool sol[Pathfinding::size][Pathfinding::size];
    obsMap.update(vectorOfPoints);
    std::cout << "rx: " << obsMap.robotX << ", ry: " << obsMap.robotY << std::endl;
    Pathfinding::getMapObjSol(sol, vectorOfPoints);
    REQUIRE(Pathfinding::boolMapsEquals(obsMap.obstacle_map, sol));

    obsMap.robotX = 5.0f;
    obsMap.robotY = 5.0f;
    std::cout << "rx: " << obsMap.robotX << ", ry: " << obsMap.robotY << std::endl;
    obsMap.update(vectorOfPoints);
    REQUIRE(!Pathfinding::boolMapsEquals(obsMap.obstacle_map, sol));

    std::vector<Point> shiftedVectorOfPoints;
    for(Point p : vectorOfPoints)
    {
        shiftedVectorOfPoints.push_back(Point{p.x - 5.0f, p.y - 5.0f});
    }
    Pathfinding::getMapObjSol(sol, shiftedVectorOfPoints);
    std::cout << "obsMap" << std::endl;
    obsMap.print();
    std::cout << "sol" << std::endl;
    Pathfinding::print(sol);
    REQUIRE(Pathfinding::boolMapsEquals(obsMap.obstacle_map, sol));
}