#include "Autonomous.h"
#include "Constants.h"
#include "Globals.h"
#include "Util.h"
#include "log.h"
#include "navtypes.h"
#include "network/MissionControlProtocol.h"
#include "rospub.h"
#include "world_interface/world_interface.h"

#include <array>
#include <chrono>
#include <csignal>
#include <ctime>
#include <fstream>
#include <sstream>
#include <thread>
#include <time.h>
#include <unistd.h>

#include <argparse/argparse.hpp>
#include <frozen/unordered_set.h>
#include <frozen/string.h>
#include <sys/time.h>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::steady_clock;
using namespace navtypes;
using namespace robot::types;

void closeRover(int signum) {
	rospub::shutdown();
	Globals::websocketServer.stop();
	raise(SIGTERM);
}

std::vector<URCLegGPS> parseGPSLegs(std::string filepath) {
	std::vector<URCLegGPS> urc_legs;
	std::ifstream gps_legs(filepath);

	int left_post_id, right_post_id;
	double lat, lon;
	std::string line;

	// A properly formatted file has each leg on a separate line, with each line of the form
	// left_post_id right_post_id lat lon
	// Improperly formatted lines (empty lines, comments) are ignored, and text
	// after the above data on properly formatted lines is ignored
	// An example can be found at example_gps_legs.txt
	while (getline(gps_legs, line)) {
		std::istringstream line_stream(line);
		if (line_stream >> left_post_id >> right_post_id >> lat >> lon) {
			// we assume that gps already has a fix
			gpscoords_t gps = {lat, lon};
			URCLegGPS leg = {left_post_id, right_post_id, gps};
			log(LOG_INFO, "Got urc leg at lat=%f lon=%f\n", lat, lon);
			urc_legs.push_back(leg);
		}
	}
	log(LOG_INFO, "Got %d urc legs\n", urc_legs.size());

	if (urc_legs.size() == 0) {
		log(LOG_ERROR, "could not get URC legs\n");
		std::exit(EXIT_FAILURE);
	}

	return urc_legs;
}

void parseCommandLine(int argc, char** argv) {
	// *  argparse::ArgumentParser program("Rover");

	// *  program.add_argument("-p", "--peripheral")
	// *  	.help("specify the peripheral mounted on the rover")
	// *  	.default_value(std::string("none"))
	// *  	.action([](const std::string& value) {
	// *  		constexpr frozen::unordered choices = frozen::make_unordered_set<frozen::string>({"lidar", "science", "arm", "none"});
	// *  		if (choices.find(value) != choices.end()) {
	// *  			return value;
	// *  		} else {
	// *  			throw std::runtime_error("Invalid peripheral " + value);
	// *  		}
	// *  	});
	// *  try {
	// *  	program.parse_args(argc, argv);
	// *  } catch (const std::runtime_error& err) {
	// *  	std::cerr << err.what() << std::endl;
	// *  	std::cerr << program;
	// *  	std::exit(EXIT_FAILURE);
	// *  }

	
}

int main(int argc, char** argv) {
	LOG_LEVEL = LOG_INFO;
	Globals::AUTONOMOUS = false;
	Globals::websocketServer.start();
	net::mc::MissionControlProtocol mcProto(Globals::websocketServer);
	Globals::websocketServer.addProtocol(mcProto);
	robot::world_interface_init();
	rospub::init();
	// Ctrl+C doesn't stop the simulation without this line
	signal(SIGINT, closeRover);

	// Target locations for autonomous navigation
	// Eventually this will be set by communication from the base station
	std::vector<URCLegGPS> urc_legs = parseGPSLegs("../src/gps/simulator_legs.txt");
	Autonomous autonomous(urc_legs, Constants::CONTROL_HZ);
	auto roverStart = steady_clock::now();
	for (int iter = 0; /*no termination condition*/; iter++) {
		auto loopStart = steady_clock::now();
		long loopStartElapsedUsecs =
			duration_cast<microseconds>(loopStart - roverStart).count();

		int arm_base_pos = robot::getMotorPos(motorid_t::armBase).getDataOrElse(-1);
		int shoulder_pos = robot::getMotorPos(motorid_t::shoulder).getDataOrElse(-1);
		int elbow_pos = robot::getMotorPos(motorid_t::elbow).getDataOrElse(-1);
		log(LOG_DEBUG, "Time\t %d arm_base\t %d\t shoulder\t %d\t elbow\t %d \r",
			loopStartElapsedUsecs / 1000, arm_base_pos, shoulder_pos, elbow_pos);

		autonomous.autonomyIter();

		long elapsedUsecs =
			duration_cast<microseconds>(steady_clock::now() - loopStart).count();
		long desiredUsecs = 1000 * 1000 / Constants::CONTROL_HZ;
		if (desiredUsecs - elapsedUsecs > 0) {
			// We drift by approximately 1ms per second unless we
			// reduce our sleep time slightly
			usleep(desiredUsecs - elapsedUsecs - 90);
		} else {
			log(LOG_WARN, "Can't keep up with control frequency! Desired %d elapsed %d\n",
				desiredUsecs / 1000, elapsedUsecs / 1000);
		}
	}
	return EXIT_SUCCESS;
}
