#include "ObjectValidator.h"

std::vector<size_t> ObjectValidator::validate(
    std::vector<std::set<std::shared_ptr<Vec2>>> lidarObstacles) {
    // Associate each extracted landmark to the closest landmark we have
    // seen before Find the center point of each cluster

    // Use the map to get a bunch of obstacles that are "near" us

    // Make a vector the size of the input vector
    // Cycle through all the sets of points
     // min_distance = INFINITY
     // Cycle through all the known obstacles
        // calculate the distance between the two obstacles
        // if (distance < min_distance)
            // min_distance = distance
            // vector[i] = known_obstacle

    // Pass each of these pairs through a validation gate
        // Use a formula with values from the EKFSLAM filter
        // If the condition is met, they are the same obstacle, insert the id
            // into the output vector
        // Otherwise, insert the obstacle into the map and insert its new id
        // into the output vector

    // return the output vector
}