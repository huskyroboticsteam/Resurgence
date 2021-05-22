
#include <libgpsmm.h>

#include "../log.h"
#include "../simulator/world_interface.h"
#include "../simulator/utils.h"

// UW lat/long is 47.653116, -122.305619
// source: http://www.csgnetwork.com/degreelenllavcalc.html
constexpr double METERS_PER_DEG_NS = 111183.53599983045;
constexpr double METERS_PER_DEG_EW = 75124.2106730417; // Seattle, WA, USA
// constexpr double METERS_PER_DEG_EW = 69498.37410392637 // Drumheller, Alberta, CA

gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);
bool gps_ready = false;
double first_fix_lat = 0.0;
double first_fix_lon = 0.0;

point_t gpsToMeters(double lon, double lat) {
  double x = (lon - first_fix_lon) * METERS_PER_DEG_EW;
  double y = (lat - first_fix_lat) * METERS_PER_DEG_NS;
  return {x,y,0};
}

transform_t readGPS() {
  if (!gps_ready) {
    if (gps_rec.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
        log(LOG_ERROR, "No GPSD running.\n");
        return toTransform({0,0,0});
    } else {
      gps_ready = true;
    }
  }

  if (!gps_rec.waiting(1000)) {
    log(LOG_ERROR, "No GPS hardware detected.\n");
    return toTransform({0,0,0});
  }

  struct gps_data_t* newdata;

  if ((newdata = gps_rec.read()) == NULL) {
    log(LOG_ERROR, "GPS read error.\n");
    return toTransform({0,0,0});
  } else {
    if (newdata->status == STATUS_NO_FIX) {
      log(LOG_WARN, "No GPS fix.\n");
      return toTransform({0,0,0});
    } else {
      double lat = newdata->fix.latitude;
      double lon = newdata->fix.longitude;
      if (first_fix_lat == 0.0) {
        // This is our first fix
        first_fix_lat = lat;
        first_fix_lon = lon;
      }

      // TODO no heading information
      return toTransform(gpsToMeters(lon, lat));
    }
  }
}

