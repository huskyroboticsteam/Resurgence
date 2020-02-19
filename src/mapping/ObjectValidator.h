#include <vector>
#include <set>
#include <memory>
#include "Cluster.h"
#include "EnvMap.h"
#include "EKFSlam.h"

/*
Used to decide which obstacles we have seen before and which we haven't
The EKFSLAM algorithm relies on knowing this information
*/
class ObjectValidator {
public:
   // Assigns ids to the lidarObstacles
   // Adds any objects that don't exist on the map to the map
   // Takes in Map obstacles from bounding box class
   std::vector<size_t> validate(std::vector<std::set<std::shared_ptr<Vec2>>> lidarObstacles); 
   ObjectValidator(EKFSLam &ekf);
private:
   EnvMap &map;
   EKFSlam &ekf;
};