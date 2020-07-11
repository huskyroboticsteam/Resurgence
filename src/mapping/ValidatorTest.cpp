#include <catch2/catch.hpp>
#include "ObjectValidator.h"
#include "EKFSlam/EKFSlam.h"

TEST_CASE("test bounding box", "[validator]")
{
    std::vector<std::vector<PointXY>> lidarClusters;
    std::vector<PointXY> cluster{PointXY{0,2}, PointXY{2,2}};
    lidarClusters.push_back(cluster);
    EKFSlam ekf;
    ObjectValidator validator(ekf);
    std::vector<PointXY> result = validator.boundingBox(lidarClusters,1);
    REQUIRE(result[0].y == 2);
    REQUIRE(result[0].x == 0);
    REQUIRE(result[1].y == 2);
    REQUIRE(result[1].x == 2);
}