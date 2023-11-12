#include "Constants.h"
#include "Globals.h"
#include "navtypes.h"
#include "network/MissionControlProtocol.h"
#include "rospub.h"
#include "world_interface/world_interface.h"

#include <array>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <loguru.cpp>
#include <sstream>
#include <thread>
#include <time.h>
#include <unistd.h>

#include <argparse/argparse.hpp>
#include <frozen/string.h>
#include <frozen/unordered_set.h>
#include <sys/time.h>

using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::steady_clock;
using namespace navtypes;
using namespace robot::types;

void closeRover(int signum) {
	robot::emergencyStop();
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
			LOG_F(INFO, "Got urc leg at lat=%f lon=%f", lat, lon);
			urc_legs.push_back(leg);
		}
	}
	LOG_F(INFO, "Got %ld urc legs\n", urc_legs.size());

	if (urc_legs.size() == 0) {
		LOG_F(ERROR, "could not get URC legs");
		std::exit(EXIT_FAILURE);
	}

	return urc_legs;
}

void parseCommandLine(int argc, char** argv) {
	argparse::ArgumentParser program("Rover", "N/A");

	program.add_argument("-p", "--peripheral")
		.help("specify the peripheral mounted on the rover")
		.default_value(std::string("none"))
		.action([](const std::string& value) {
			std::unordered_map<std::string, mountedperipheral_t> allowed{
				{"none", mountedperipheral_t::none},
				{"arm", mountedperipheral_t::arm},
				{"armServo", mountedperipheral_t::armServo},
				{"science", mountedperipheral_t::scienceStation},
				{"lidar", mountedperipheral_t::lidar}};

			if (allowed.find(value) != allowed.end()) {
				Globals::mountedPeripheral = allowed[value];
				return value;
			}

			throw std::runtime_error("Invalid peripheral " + value);
		});

	program.add_argument("-llvl", "--loglevel")
		.help("specify the log level to use (must be one of: LOG_TRACE, LOG_DEBUG, LOG_INFO, "
			  "LOG_WARN, LOG_ERROR)")
		.default_value(std::string("LOG_INFO"))
		.action([](const std::string& value) {
			std::unordered_map<std::string, int> allowed{
				{"LOG_ERROR", loguru::Verbosity_ERROR},
				{"LOG_WARN", loguru::Verbosity_WARNING},
				{"LOG_INFO", loguru::Verbosity_INFO},
				{"LOG_DEBUG", 2},
				{"LOG_TRACE", 1},
			};

			if (allowed.find(value) != allowed.end()) {
				loguru::g_stderr_verbosity = allowed[value];
			}

			throw std::runtime_error("Invalid log level " + value);
		});

	program.add_argument("-nc", "--no-colors")
		.help("disables colors in console logging")
		.action([&](const auto&) { loguru::g_colorlogtostderr = false; })
		.nargs(0);

	try {
		loguru::init(argc, argv);
		namespace fs = std::filesystem;

		const int LOG_LIFESPAN = 604800; // 7 days
		try {
			for (const auto& entry : fs::directory_iterator(fs::current_path())) {
				std::cout << entry.path().extension();
				if (entry.path().extension() == ".log" && entry.path().stem() != "latest") {
					std::string dateString = entry.path().stem();

					// Extract components from the date string
					int year, month, day, hour, minute, second;
					sscanf(dateString.c_str(), "%4d%2d%2d_%2d%2d%2d", &year, &month, &day,
						   &hour, &minute, &second);

					// Create a tm structure
					std::tm tm_struct = {};
					tm_struct.tm_year = year - 1900;
					tm_struct.tm_mon = month - 1;
					tm_struct.tm_mday = day;
					tm_struct.tm_hour = hour;
					tm_struct.tm_min = minute;
					tm_struct.tm_sec = second;

					// Current time
					std::time_t unixTime = std::mktime(&tm_struct);

					// Delete log if it's older than LOG_LIFESPAN
					if (std::chrono::system_clock::to_time_t(
							std::chrono::system_clock::now()) -
							unixTime >
						LOG_LIFESPAN) {
						fs::remove(entry.path());
					}
				}
			}
		} catch (const fs::filesystem_error& e) {
			std::cerr << "Error accessing current directory!" << e.what() << std::endl;
		}

		const auto now = std::chrono::system_clock::now();
		const std::time_t t_c = std::chrono::system_clock::to_time_t(now);
		std::stringstream ss;
		ss << std::put_time(std::localtime(&t_c), "%Y%m%d_%H%M%S.log");
		std::string logFileName = ss.str();
		loguru::add_file("latest.log", loguru::Truncate, loguru::Verbosity_INFO);
		loguru::add_file(logFileName.c_str(), loguru::Append, loguru::Verbosity_INFO);
		LOG_F(INFO, "Logging to %s", logFileName.c_str());

		program.parse_args(argc, argv);
		LOG_F(INFO,
			  "parseCommandLine got peripheral specified as: \"%s\", logLevel specified as: "
			  "\"%s\"",
			  program.get<std::string>("peripheral").c_str(),
			  program.get<std::string>("loglevel").c_str());
	} catch (const std::runtime_error& err) {
		std::cerr << err.what() << std::endl;
		std::cerr << program;
		std::exit(EXIT_FAILURE);
	}
}

int main(int argc, char** argv) {
	parseCommandLine(argc, argv);
	Globals::AUTONOMOUS = false;
	Globals::websocketServer.start();
	robot::world_interface_init();
	auto mcProto = std::make_unique<net::mc::MissionControlProtocol>(Globals::websocketServer);
	Globals::websocketServer.addProtocol(std::move(mcProto));
	rospub::init();
	// Ctrl+C doesn't stop the simulation without this line
	signal(SIGINT, closeRover);

	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(60));
	}
}
