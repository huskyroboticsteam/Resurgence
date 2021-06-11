
#include "../simulator/utils.h"

void world_interface_init(); // Call this before trying to do anything else

// If the requested dtheta/dx is too fast for the robot to execute, it will
// scale them down and return the corresponding scale factor.
double setCmdVel(double dtheta, double dx);

points_t readLidarScan();
points_t readLandmarks();
transform_t readGPS();
transform_t readOdom();
point_t gpsToMeters(double lon, double lat);

// `index` must be in the range 0-6 (the URC competition will have 7 legs)
URCLeg getLeg(int index);
