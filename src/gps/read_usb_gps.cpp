
#include <libgpsmm.h>
#include <thread>
#include <mutex>

#include "../log.h"
#include "../simulator/world_interface.h"
#include "../simulator/utils.h"

// UW lat/long is 47.653116, -122.305619
// source: http://www.csgnetwork.com/degreelenllavcalc.html
constexpr double METERS_PER_DEG_NS = 111183.53599983045;
constexpr double METERS_PER_DEG_EW = 75124.2106730417; // Seattle, WA, USA
// constexpr double METERS_PER_DEG_EW = 69498.37410392637 // Drumheller, Alberta, CA

gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);
std::thread gps_thread;
std::mutex gps_mutex;
double first_fix_lat = 0.0;
double first_fix_lon = 0.0;
bool fresh_data = false;
transform_t most_recent_tf;

bool gpsHasFix() {
  gps_mutex.lock();
  bool r = first_fix_lat != 0.0;
  gps_mutex.unlock();
  return r;
}

bool gpsHasFreshData() {
  gps_mutex.lock();
  bool r = fresh_data;
  gps_mutex.unlock();
  return r;
}

point_t gpsToMeters__private(double lon, double lat) {
  double x = (lon - first_fix_lon) * METERS_PER_DEG_EW;
  double y = (lat - first_fix_lat) * METERS_PER_DEG_NS;
  return {x,y,0};
}

point_t gpsToMeters(double lon, double lat) {
  gps_mutex.lock();
  point_t r = gpsToMeters__private(lon, lat);
  gps_mutex.unlock();
  return r;
}

transform_t readGPS() {
  gps_mutex.lock();
  transform_t tf = most_recent_tf;
  gps_mutex.unlock();
  return tf;
}

void gps_loop() {
  struct gps_data_t* newdata;

  while (true) {
    if (!gps_rec.waiting(5000000)) {
      log(LOG_ERROR, "Lost connection to GPS hardware!\n");
      continue;
    }

    if ((newdata = gps_rec.read()) == NULL) {
      log(LOG_ERROR, "GPS read error.\n");
      continue;
    } else {
      if (std::isnan(newdata->fix.latitude)) {
        log(LOG_WARN, "No GPS fix.\n");
        continue;
      } else {
        double lat = newdata->fix.latitude;
        double lon = newdata->fix.longitude;
        point_t p = gpsToMeters__private(lon, lat);
        log(LOG_DEBUG, "Received fresh GPS data: %.2f %.2f (lat %.6f, lon %.6f).\n",
            p(0), p(1), lat, lon);
        gps_mutex.lock();
        if (first_fix_lat == 0.0) {
          // This is our first fix
          first_fix_lat = lat;
          first_fix_lon = lon;
        }

        fresh_data = true;
        // TODO no heading information
        most_recent_tf = toTransform(p);
        gps_mutex.unlock();
      }
    }
  }
}

bool startGPSThread() {
  if (gps_rec.stream(WATCH_ENABLE|WATCH_JSON) == NULL) {
    log(LOG_ERROR, "No GPSD running.\n");
    return false;
  }
  if (!gps_rec.waiting(5000000)) {
    log(LOG_ERROR, "No GPS hardware detected.\n");
    return false;
  }
  gps_thread = std::thread(gps_loop);
  return true;
}