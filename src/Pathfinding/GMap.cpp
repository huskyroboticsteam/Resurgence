#include "GMap.h"

class GMap
{

    // GMap()
    // {

    // }

    void getRobotPosition(int& x, int& y)
    {
        Point p = ekfslamobj.getPosition();
        p.x = x;
        p.y = y;
    }

}