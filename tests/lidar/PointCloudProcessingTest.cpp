#define CATCH_CONFIG_MAIN

#include "lidar/PointCloudProcessing.h"

#include <catch2/catch.hpp>

namespace Lidar
{
    std::vector<std::shared_ptr<PointXY>> getPointsInRadius(float x0, float y0, float r, 
        int numPts)
    {
        std::vector<std::shared_ptr<PointXY>> pts;
        for (int i = 0; i < numPts; i++)
        {
            float x = x0 - r + static_cast<float>(rand()) /
                (static_cast<float>(RAND_MAX / (2 * r)));
            float yAbsMax = sqrtf(powf(r, 2) - powf(x - x0, 2));
            float y = y0 - yAbsMax + static_cast<float>(rand() /
                (static_cast<float>(RAND_MAX / (2 * yAbsMax))));
            PointXY p;
            p.x = x;
            p.y = y;
            pts.push_back(std::make_shared<PointXY>(p));
        }
        return pts;
    }
}

TEST_CASE("cluster many nearby points into 1 cluster")
{
    using namespace Lidar;
    std::vector<std::shared_ptr<PointXY>> pts = getPointsInRadius(0, 0, 1.0, 10);
    std::vector<std::set<std::shared_ptr<PointXY>>> clusters = clusterPoints(pts, 1.0);
    REQUIRE(clusters.size() == 1);
    REQUIRE(clusters[0].size() == 10);
}
