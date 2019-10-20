#define CATCH_CONFIG_MAIN

#include "mapping/Map2D.h"

#include <iostream>
#include <cmath>
#include <chrono>
#include <ctime>
#include <thread>

constexpr int test_iterations = 100;
constexpr int vertex_count = 1000;
constexpr int sparsity_factor = 2;

#include <catch2/catch.hpp>

namespace Mapping
{

// creates a map with given number of vertices
// when connecting neighbors, the probability that any node will be connected to any other node
// is 1 / sparse_factor
// thus if sparse_factor is 1, each node is connected to each other node
std::shared_ptr<Map2D> CreateRandMap(int vertices, int sparse_factor)
{
    Map2D map;
    for (int i = 0; i < vertices; i++)
    {
        Point2D p;
        p.id = i;
        p.x = (float)(rand() % vertices);
        p.y = (float)(rand() % vertices);
        std::shared_ptr<Point2D> p_ptr = std::make_shared<Point2D>(p);
        for (int j = 0; j < map.vertices.size(); j++)
        {
            if (rand() % sparse_factor == 0)
            {
                Connect(p_ptr, map.vertices.at(j));
            }
        }
        map.vertices.push_back(p_ptr);
    }
    return std::make_shared<Map2D>(map);
}

// randomly picks a start and end vertex, making sure they are different nodes
// returns average time of shortest path computation, in seconds
double TimeShortestPath(std::shared_ptr<Map2D> map, int num_iterations)
{
    double avg = 0;
    for (int i = 0; i < num_iterations; i++)
    {
        int start_ind = rand() % map->vertices.size();
        int target_ind = rand() % map->vertices.size();
        while (target_ind == start_ind)
        {
            target_ind = rand() % map->vertices.size();
        }

        auto start = std::chrono::system_clock::now();
        ComputePath(map, map->vertices[start_ind], map->vertices[target_ind]);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        avg += elapsed_seconds.count() / num_iterations;
    }
    return avg;
}

bool verifyCorrectPath()
{
    Map2D map;
    float x_pts[10] = {0, -1, 2, -3, 4, -5, 6, -7, 8, -9};
    float y_pts[10] = {0, -1, 2, -3, 4, -5, 6, -7, 8, -9};
    for (int i = 0; i < 10; i++)
    {
        Point2D p;
        p.id = i;
        p.x = x_pts[i];
        p.y = y_pts[i];
        map.vertices.push_back(std::make_shared<Point2D>(p));
        if (i > 0)
        {
            Connect(map.vertices[i], map.vertices[i - 1]);
        }
    }
    Connect(map.vertices[9], map.vertices[2]);
    std::vector<std::shared_ptr<Point2D>> actual = ComputePath(std::make_shared<Map2D>(map),
                                                               map.vertices[9], map.vertices[0]);
    int expected[3] = {0, 1, 2};
    for (int i = 0; i < actual.size(); i++)
    {
        if (expected[i] != actual[i]->id)
        {
            return false;
        }
    }
    return true;
}
} // namespace Mapping

TEST_CASE("computes correct path")
{
    REQUIRE(Mapping::verifyCorrectPath());
}
