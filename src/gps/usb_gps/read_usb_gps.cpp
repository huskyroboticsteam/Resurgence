#include "read_usb_gps.h"

#include "../../navtypes.h"
#include "../../world_interface/world_interface.h"
#include "../gps_util.h"

#include <libgpsmm.h>
#include <loguru.hpp>
#include <mutex>
#include <optional>
#include <thread>
#include <unistd.h>

using namespace robot::types;

namespace gps {
gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);
std::thread gps_thread;
std::mutex gps_mutex;
bool has_fix;
navtypes::gpscoords_t most_recent_coords;
datatime_t gps_time;

// implements method from world_interface.h
DataPoint<navtypes::gpscoords_t> readGPSCoords() {
	DataPoint<navtypes::gpscoords_t> data;
	gps_mutex.lock();
	if (robot::gpsHasFix()) {
		data = DataPoint<navtypes::gpscoords_t>(gps_time, most_recent_coords);
	}
	gps_mutex.unlock();
	return data;
}

namespace usb {
bool gpsHasFix() {
	gps_mutex.lock();
	bool r = has_fix;
	gps_mutex.unlock();
	return r;
}

void gps_loop() {
	struct gps_data_t* newdata;
	loguru::set_thread_name("USB_GPS");

	while (true) {
		if (!gps_rec.waiting(5000000)) {
			LOG_F(ERROR, "Lost connection to GPS hardware!");
			continue;
		}

		if ((newdata = gps_rec.read()) == NULL) {
			LOG_F(ERROR, "GPS read error.");
			continue;
		} else if (newdata->set & STATUS_SET) {
			if (!(newdata->set & LATLON_SET) || std::isnan(newdata->fix.latitude)) {
				LOG_F(WARN, "No GPS fix.");
				continue;
			} else {
				double lat = newdata->fix.latitude;
				double lon = newdata->fix.longitude;
				LOG_F(2, "Received fresh GPS data: (lat %.6f, lon %.6f).", lat, lon);
				gps_mutex.lock();
				if (!has_fix) {
					// This is our first fix
					has_fix = true;
				}

				most_recent_coords = {lat, lon};
				gps_time = dataclock::now();
				gps_mutex.unlock();
			}
		} else {
			LOG_F(WARNING, "Got packet with no STATUS_SET.");
		}
	}
}

bool detectHardware() {
	struct gps_data_t* packet;

	int attempts = 0;
	while (attempts++ < 10) {
		if (!gps_rec.waiting(5000000)) {
			return false;
		} else if ((packet = gps_rec.read()) == NULL) {
			LOG_F(ERROR, "GPS read error.");
		} else if (packet->set & STATUS_SET) {
			return true;
		}
		usleep(100 * 1000);
	}
	return false;
}

bool startGPSThread() {
	if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
		LOG_F(ERROR, "No GPSD running.");
		return false;
	}
	if (!detectHardware()) {
		LOG_F(ERROR, "No GPS hardware detected. (If not using GPS, please re-run "
					 "enable_CAN_and_GPS.sh to stop gpsd.)");
		return false;
	}
	gps_thread = std::thread(gps_loop);
	return true;
}
} // namespace usb
} // namespace gps
