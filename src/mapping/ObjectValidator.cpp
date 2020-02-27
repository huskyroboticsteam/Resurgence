#include "ObjectValidator.h"

// Associate each extracted landmark to the closest landmark we have seen before 
std::vector<size_t> ObjectValidator::validate(std::vector<std::set<std::shared_ptr<PointXY>>> lidarClusters) {
    std::vector<PointXY> lidarObstacles = boundingBox(lidarClusters,1);
    const int landmarkConstant = 1;    //placeholder value for now
    const float lidarRange = 10;       //temp lidar range
    std::vector<size_t> obstacleIDs;
    PointXY currPosition = ekf.getPosition(); 

    // Use the map to get all known obstacles
    std::vector<ObstaclePoint> existingObs = ekf.getObstacles();

    // Cycle through all the passed obstacles
    for(int i = 0; i < lidarObstacles.size; i++) 
    {
        float leastDistance = lidarRange; //start value at greater than range of lidar
        PointXY closestObstacle;
        int id;
        // Cycle through all known obstacles 
        for(int j = 0; j < existingObs.size; j++) 
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

std::vector<PointXY> boundingBox(std::vector<std::set<std::shared_ptr<PointXY>>> lidarClusters, float boxSize) {
    std::vector<PointXY> boxes;
    float boxRadius = boxSize/2;
    //Loop through every set of clusters from the lidar
    //Each set is a seperate obstacles
    for(set<std::shared_ptr<PointXY>> clusterSet: lidarClusters) {
        std::set<std::shared_ptr<PointXY>>::iterator it = clusterSet.begin();
        PointXY firstPoint = **it;
        boxes.push_back(firstPoint); //put a "box" around the first point
        it++;
        //iterate through the set
        while (it != clusterSet.end()) {
            PointXY point = **it;
            //check if point in set is within any of the existing boxes
            for(PointXY box : boxes) {
                bool inX = (point.x > (box.x - boxRadius)) && (point.x < (box.x + boxRadius));
                bool inY = (point.y > (box.y - boxRadius)) && (point.y < (box.y + boxRadius));
                if(!inX || !inY) {
                    //if the point is not in any boxes, add it as a box
                    boxes.push_back(point);
                }
            }
	        //Increment the iterator
        	it++;
        }
    }
    return boxes;
}
        
