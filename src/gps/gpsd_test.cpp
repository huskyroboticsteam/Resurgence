/* Note from Nathan Hatch (Husky Robotics):
 *
 * This code is copied from https://github.com/jcable/gpsd/blob/master/test_gpsmm.cpp
 * and modified slightly. (There was a change in how the gps_data_t struct stores
 * the information about the satellites.)
 */

/*
 * Copyright (C) 2010 Eric S. Raymond.
 *
 * This software is distributed under a BSD-style license. See the
 * file "COPYING" in the top-level directory of the distribution for details.
 *
 */

/* This simple program shows the basic functionality of the C++ wrapper class */
#include <iostream>
#include <libgpsmm.h>

using namespace std;

/*
 * We should get libgps_dump_state() from the client library, but
 * scons has a bug; we can't get it to add -lgps to the link line,
 * apparently because it doesn't honor parse_flags on a Program()
 * build of a C++ file.
 */
static void libgps_dump_state(struct gps_data_t* collect) {
	/* no need to dump the entire state, this is a sanity check */
#ifndef USE_QT
	/* will fail on a 32-bit machine */
	(void)fprintf(stdout, "flags: (0x%04x) %s\n", (unsigned int)collect->set,
				  gps_maskdump(collect->set));
#endif
	if (collect->set & ONLINE_SET)
		(void)fprintf(stdout, "ONLINE: %ld.%09ld\n", collect->online.tv_sec,
					  collect->online.tv_nsec);
	if (collect->set & TIME_SET)
		(void)fprintf(stdout, "TIME: %ld.%09ld\n", collect->fix.time.tv_sec,
					  collect->online.tv_nsec);
	if (collect->set & LATLON_SET)
		(void)fprintf(stdout, "LATLON: lat/lon: %lf %lf\n", collect->fix.latitude,
					  collect->fix.longitude);
	if (collect->set & ALTITUDE_SET)
		(void)fprintf(stdout, "ALTITUDE: altitude: %lf  U: climb: %lf\n",
					  collect->fix.altitude, collect->fix.climb);
	if (collect->set & SPEED_SET)
		(void)fprintf(stdout, "SPEED: %lf\n", collect->fix.speed);
	if (collect->set & TRACK_SET)
		(void)fprintf(stdout, "TRACK: track: %lf\n", collect->fix.track);
	if (collect->set & CLIMB_SET)
		(void)fprintf(stdout, "CLIMB: climb: %lf\n", collect->fix.climb);
	if (collect->set & STATUS_SET)
		// (void)fprintf(stdout, "STATUS: status: %d\n", collect->status);
		if (collect->set & MODE_SET)
			(void)fprintf(stdout, "MODE: mode: %d\n", collect->fix.mode);
	if (collect->set & DOP_SET)
		(void)fprintf(stdout, "DOP: satellites %d, pdop=%lf, hdop=%lf, vdop=%lf\n",
					  collect->satellites_used, collect->dop.pdop, collect->dop.hdop,
					  collect->dop.vdop);
	if (collect->set & VERSION_SET)
		(void)fprintf(stdout, "VERSION: release=%s rev=%s proto=%d.%d\n",
					  collect->version.release, collect->version.rev,
					  collect->version.proto_major, collect->version.proto_minor);
	if (collect->set & POLICY_SET)
		(void)fprintf(stdout,
					  "POLICY: watcher=%s nmea=%s raw=%d scaled=%s timing=%s, devpath=%s\n",
					  collect->policy.watcher ? "true" : "false",
					  collect->policy.nmea ? "true" : "false", collect->policy.raw,
					  collect->policy.scaled ? "true" : "false",
					  collect->policy.timing ? "true" : "false", collect->policy.devpath);
	if (collect->set & SATELLITE_SET) {
		int i;

		(void)fprintf(stdout, "SKY: satellites in view: %d\n", collect->satellites_visible);
		for (i = 0; i < collect->satellites_visible; i++) {
			(void)fprintf(stdout, "    %2.2d: %2.2f %3.3f %3.0f %c\n", collect->skyview[i].PRN,
						  collect->skyview[i].elevation, collect->skyview[i].azimuth,
						  collect->skyview[i].ss, collect->skyview[i].used ? 'Y' : 'N');
		}
	}
	if (collect->set & DEVICE_SET)
		(void)fprintf(stdout, "DEVICE: Device is '%s', driver is '%s'\n", collect->dev.path,
					  collect->dev.driver);
#ifdef OLDSTYLE_ENABLE
	if (collect->set & DEVICEID_SET)
		(void)fprintf(stdout, "GPSD ID is %s\n", collect->dev.subtype);
#endif /* OLDSTYLE_ENABLE */
	if (collect->set & DEVICELIST_SET) {
		int i;
		(void)fprintf(stdout, "DEVICELIST:%d devices:\n", collect->devices.ndevices);
		for (i = 0; i < collect->devices.ndevices; i++) {
			(void)fprintf(stdout, "%d: path='%s' driver='%s'\n", collect->devices.ndevices,
						  collect->devices.list[i].path, collect->devices.list[i].driver);
		}
	}
}

int main(void) {
	gpsmm gps_rec("localhost", DEFAULT_GPSD_PORT);

	if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
		cerr << "No GPSD running.\n";
		return 1;
	}

	for (;;) {
		struct gps_data_t* newdata;

		if (!gps_rec.waiting(5000000))
			continue;

		if ((newdata = gps_rec.read()) == NULL) {
			cerr << "Read error.\n";
			return 1;
		} else {
			libgps_dump_state(newdata);
		}
	}

	cout << "Exiting\n";
	return 0;
}
