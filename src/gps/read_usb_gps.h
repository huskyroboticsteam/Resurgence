
#pragma once

#include "../simulator/utils.h"

bool gpsHasFix();
bool gpsHasFreshData();
point_t gpsToMeters(double lon, double lat);
bool startGPSThread();
