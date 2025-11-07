#include "Constants.h"
#include "Globals.h"
#include "navtypes.h"
#include "network/MissionControlProtocol.h"
#include "world_interface/world_interface.h"
#include "ar/read_landmarks.h"

#include <array>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
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

void closeRover(int) {
	robot::emergencyStop();
	Globals::websocketServer.stop();
	raise(SIGTERM);
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
				{"science", mountedperipheral_t::science}};

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
				{"LOG_VERBOSE", 1},
				{"LOG_TRACE", 2},
				{"LOG_DEBUG", 3},
			};

			if (allowed.find(value) != allowed.end()) {
				loguru::g_stderr_verbosity = allowed[value];
				return value;
			} else {
				throw std::runtime_error("Invalid log level " + value);
			}
		});

	program.add_argument("-nc", "--no-colors")
		.help("disables colors in console logging")
		.action([&](const auto&) { loguru::g_colorlogtostderr = false; })
		.nargs(0);

	try {
		loguru::init(argc, argv);
		namespace fs = std::filesystem;

		const auto LOG_LIFESPAN = std::chrono::hours(24 * 7);
		try {
			for (const auto& entry : fs::directory_iterator(fs::current_path())) {
				if (entry.path().extension() == ".log" && entry.path().stem() != "latest") {
					// Format of dateString: YYYYMMDD_HHMMSS
					std::string dateString = entry.path().stem();

					// Extract components from the date string
					int year, month, day, hour, minute, second;
					int scan_count = sscanf(dateString.c_str(), "%4d%2d%2d_%2d%2d%2d", &year,
											&month, &day, &hour, &minute, &second);

					// Ensure scan output count is 6
					CHECK_EQ_F(scan_count, 6);

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

					// Calculate duration
					auto duration = std::chrono::system_clock::now() -
									std::chrono::system_clock::from_time_t(unixTime);

					// Delete log if it's older than LOG_LIFESPAN
					if (duration > LOG_LIFESPAN) {
						fs::remove(entry.path());
					}
				}
			}
		} catch (const fs::filesystem_error& e) {
			LOG_F(ERROR, "Error accessing current directory! %s", e.what());
		}

		const auto now = std::chrono::system_clock::now();
		const std::time_t t_c = std::chrono::system_clock::to_time_t(now);
		std::stringstream ss;
		ss << "logs/" << std::put_time(std::localtime(&t_c), "%Y%m%d_%H%M%S.log");
		std::string logFileName = ss.str();
		loguru::add_file("logs/latest.log", loguru::Truncate, loguru::Verbosity_INFO);
		loguru::add_file(logFileName.c_str(), loguru::Append, loguru::Verbosity_INFO);

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
	robot::world_interface_init(Globals::websocketServer);
	auto mcProto = std::make_unique<net::mc::MissionControlProtocol>(Globals::websocketServer);
	Globals::websocketServer.addProtocol(std::move(mcProto));
	// Ctrl+C doesn't stop the simulation without this line
	signal(SIGINT, closeRover);
	
	// Open mast camera to load its configuration before initializing AR
	LOG_F(INFO, "Opening mast camera...");
	auto mastCam = robot::openCamera(Constants::MAST_CAMERA_ID);
	
	// Initialize AR landmark detection
	LOG_F(INFO, "Initializing AR landmark detection...");
	if (AR::initializeLandmarkDetection()) {
		LOG_F(INFO, "AR landmark detection initialized successfully");
	} else {
		LOG_F(WARNING, "Failed to initialize AR landmark detection");
	}

	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(60));
	}
}
