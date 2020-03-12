#include "ObjectValidator.h"
#include "EKFSlam/EKFSlam.h"

// Associate each extracted landmark to the closest landmark we have seen before 
std::vector<size_t> ObjectValidator::validate(std::vector<PointXY>& lidarObstacles) {
    
    const int landmarkConstant = 1;    //placeholder value for now
    const float lidarRange = 10;       //temp lidar range
    std::vector<size_t> obstacleIDs;
    PointXY currPosition = ekf.getPosition(); 

    // Use the map to get all known obstacles
    std::vector<ObstaclePoint> existingObs = ekf.getObstacles();

    // Cycle through all the passed obstacles
    for(int i = 0; i < lidarObstacles.size(); i++) 
    {
        float leastDistance = lidarRange; //start value at greater than range of lidar
        PointXY closestObstacle;
        int id;
        // Cycle through all known obstacles 
        for(int j = 0; j < existingObs.size(); j++) 
        {
            // calculate the distance between the two obstacles
            float deltaX = existingObs[j].x - lidarObstacles[i].x;
            float deltaY = existingObs[j].y - lidarObstacles[i].y;
            float distance = std::sqrt(std::pow(deltaX , 2.0) + std::pow(deltaY , 2.0));
            //Find pair of closest obstacles;
            if(distance < leastDistance) 
            {
                leastDistance = distance;
                closestObstacle.x = existingObs[j].x;
                closestObstacle.y = existingObs[j].y;
                id = j + 1; //associate passed obstacle with an id of a known obstacle
            }
        }
        // pass id of closest obstacle and the map obstacle we've associated with it 
        // get back validation value to see if obstacles are the same
        float validationValue = ekf.getValidationValue(id, lidarObstacles[i]);
        if (validationValue <= landmarkConstant) 
        { //passed validation gate
            obstacleIDs.push_back(id);        //add id of known obstacle
        }
        else 
        { // failed validation gate, is a new obstacle
            int newID = ekf.getNewLandmarkID();  //add new obstacle to map, get an id 
            obstacleIDs.push_back(newID);  //add id to vector
        }
    }
    return obstacleIDs; // return the output vector
}

std::vector<PointXY> ObjectValidator::boundingBox(std::vector<std::vector<PointXY>> lidarClusters, float boxSize) {
    std::vector<PointXY> boxes;
    float boxRadius = boxSize/2;
    //Loop through every vector of clusters from the lidar
    //Each vector is a seperate obstacle
    for(int i = 0; i < lidarClusters.size(); i++) {
        PointXY firstPoint = lidarClusters[i][0];
        boxes.push_back(firstPoint); //put a box around the first point of an obstacle
        int prevBoxes = boxes.size(); //find box index to start at
        //loop through each point in an obstacle
        for(int j = 1; j < lidarClusters[i].size(); j++) {
            //check if point in obstacle is within any of the existing boxes
            //will stop looping if point is in a box or we have looped through all the boxes
            bool inBox = false;
            for(int k = prevBoxes - 1; (k < boxes.size()) && (inBox == false); k++) {
                bool inX = (lidarClusters[i][j].x > (boxes[k].x - boxRadius)) && 
                           (lidarClusters[i][j].x < (boxes[k].x + boxRadius));
                bool inY = (lidarClusters[i][j].y > (boxes[k].y - boxRadius)) && 
                           (lidarClusters[i][j].y < (boxes[k].y + boxRadius));
                if(inX && inY) {
                    //point is within an existing box, stop looping
                    inBox = true;
                } 
            }
            if(!inBox) {
                //done looping, point isn't in a box, so add it as one
                boxes.push_back(lidarClusters[i][j]);
            }
        } 
    }
    return boxes;
}
        
