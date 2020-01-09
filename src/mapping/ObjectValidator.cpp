#include "ObjectValidator.h"

// Associate each extracted landmark to the closest landmark we have seen before 
std::vector<size_t> ObjectValidator::validate(std::vector<MapObstacle> lidarObstacles) {

    const int landmarkConstant = 1;    //placeholder value for now
    const float lidarRange = 10;       //temp lidar range
    std::vector<size_t> obstacleIDs;
    Vec2 currPosition = ekf.getPosition(); 

    // Use the map to get a bunch of known obstacles near our current position
    std::vector<std::shared_ptr<const MapObstacle>> nearObstacles = 
        map.findObjectsWithinRadius(lidarRange, currPosition.x, currPosition.y);

    // Cycle through all the passed obstacles
    for(int i = 0; i < lidarObstacles.size; i++) 
    {
        float leastDistance = lidarRange; //start value at greater than range of lidar
        MapObstacle closestObstacle;
        // Cycle through all known obstacles 
        for(std::shared_ptr<const MapObstacle> obstacle : nearObstacles) 
        {
            MapObstacle ob = *obstacle;
            Vec2 obstaclePos = ob.position;
            // calculate the distance between the two obstacles
            float deltaX = obstaclePos.x - lidarObstacles[i].position.x;
            float deltaY = obstaclePos.y - lidarObstacles[i].position.y;
            float distance = std::sqrt(std::pow(deltaX , 2.0) + std::pow(deltaY , 2.0));
            //Find pair of closest obstacles;
            if(distance < leastDistance) 
            {
                leastDistance = distance;
                closestObstacle = ob;
            }
        }
        // pass id of closest obstacle and the map obstacle we've associated with it, get back value
        float validationValue = ekf.getValidationValue(closestObstacle.uid, lidarObstacles[i]);
        if (validationValue <= landmarkConstant) 
        { //passed validation gate
            obstacleIDs.push_back(ob.uid);        //add id of known obstacle
        }
        else 
        { // failed validation gate, is a new obstacle
            size_t newID = map.newObstacleUID(lidarObstacles[i]); //add new obstacle to map, get an id 
            obstacleIDs.push_back(newID);                         //add id to vector
        }
    }
    return obstacleIDs; // return the output vector
}