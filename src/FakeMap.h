#pragma once
#include <vector>
#include <iostream>
#include "../src/lidar/Cluster.h"
#include "WorldData.h"

class Autonomous;

class FakeMap : public WorldData {
    public:
        FakeMap(Autonomous& autonomous_);
        bool lidarSees() override;
        PointXY getGPS() override;
        float getHeading() override;
    private:
        void callAutonomous ();
        void update(std::pair<float, float> directions);
        std::vector<float> makeLine(float x2, float y2, float x1, float y1);
        Autonomous& autonomous;
        float heading;
        PointXY robotPos;
        std::vector<std::pair<PointXY, PointXY>> obstacles;
        
};
