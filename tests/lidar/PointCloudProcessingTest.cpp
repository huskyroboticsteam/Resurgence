#define CATCH_CONFIG_MAIN

#include "lidar/PointCloudProcessing.h"
#include "lidar/PointGenerator.h"

#include <catch2/catch.hpp>

TEST_CASE("cluster many nearby points into 1 cluster")
{
    using namespace Lidar;
    std::vector<std::shared_ptr<PointXY>> pts = generateClusterRadius(0, 0, 1, 10);
    std::vector<std::set<std::shared_ptr<PointXY>>> clusters = clusterPoints(pts, 1.0);
    REQUIRE(clusters.size() == 1);
    REQUIRE(clusters[0].size() == 10);
}
