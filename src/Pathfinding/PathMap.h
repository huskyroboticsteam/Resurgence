#pragma once
#include <vector>

// recieve list of obstacles within specified range
// plot obstacles onto 2d grid
// pass grid onto pathing
// recreate grid as orientation/location changes

//increase bounding box 

class PathMap{
    vector< vector<bool> > pathMap;

private:
    void getData();
    
public:
    void updatePathMap();
    void PathMap();
}