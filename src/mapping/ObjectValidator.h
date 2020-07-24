#pragma once

#include <vector>
#include <set>
#include <memory>
#include "../lidar/PointCloudProcessing.h"

#include "../math/PointXY.h"

class EKFSlam;

/*
Used to decide which obstacles we have seen before and which we haven't
The EKFSLAM algorithm relies on knowing this information
*/

class ObjectValidator {
public:
   //Associates passed obstacles to lidarObstacles
   //Assigns the correct id to associated obstacles
   //Gives a new id to obstacles considered to be new
   //Returns a vector of those ids
   std::vector<size_t> validate(std::vector<PointXY>& lidarClusters); 
   ObjectValidator(EKFSlam &ekf_) : ekf(ekf_) {}
   //Takes in clusters of points collected from the lidar
   //Associates a point as an obstacle, putting a "box" around it
   //Treat any point not within a box as also a new obstacle
   //Returns a vector of points of all the obstacles
   //The point represents the center of the box
   std::vector<PointXY> boundingBox(std::vector<std::vector<PointXY>> lidarClusters, float boxSize);
private:

   EKFSlam &ekf;
};
