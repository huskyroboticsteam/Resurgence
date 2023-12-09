/**
 * This file logs filter performance, which can be processed and visualized.
 */

#include "../Constants.h"
#include "../Globals.h"
#include "../navtypes.h"
#include "../world_interface/data.h"
#include "../world_interface/world_interface.h"
#include "FullPoseEstimator.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <loguru.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace robot::types;
using namespace std::chrono_literals;
using namespace navtypes;
using std::chrono::duration_cast;

void usage(const char* name);

template <typename K, typename V>
constexpr std::unordered_map<V, K> reverseMap(const std::unordered_map<K, V>& map);

enum class FilterType {
	EKF
};

const std::unordered_map<std::string, FilterType> filterNameToType{{"EKF", FilterType::EKF}};

const std::unordered_map<FilterType, std::string> filterTypeToName =
	reverseMap(filterNameToType);

int main(int argc, const char* argv[]) {
	if (argc < 3) {
		usage(argv[0]);
		return 1;
	}

	std::unordered_set<FilterType> filters;
	for (int i = 2; i < argc; i++) {
		std::string filterName(argv[i]);
		filters.insert(filterNameToType.at(filterName));
	}
	std::filesystem::path outputDir(argv[1]);

	std::filesystem::create_directories(outputDir);
	std::unordered_map<FilterType, std::ofstream> filterStreamMap;
	std::ofstream truePoseStream(outputDir / "true_pose.csv");
	truePoseStream << "Time,X,Y,Theta\n";
	for (auto filterType : filters) {
		std::filesystem::path path = outputDir / (filterTypeToName.at(filterType) + ".csv");
		filterStreamMap.emplace(filterType, std::ofstream(path));
		filterStreamMap.at(filterType) << "Time,X,Y,Theta\n";
	}

	Globals::websocketServer.start();
	robot::world_interface_init();

	filters::FullPoseEstimator fpe({1, 1}, Constants::EFF_WHEEL_BASE, 0.1, {0.05, 0.05}, 0.05);

	double xVel = 1, thetaVel = 0;
	robot::setCmdVel(thetaVel, xVel);

	std::optional<datatime_t> lastGPSTime;
	std::optional<datatime_t> lastHeadingTime;
	std::optional<datatime_t> lastTruthTime;
	datatime_t startTime = dataclock::now();
	std::cout << "Collecting data!" << std::endl;
	while ((dataclock::now() - startTime) < 30s) {
		datatime_t now = dataclock::now();
		auto truePoseDP = robot::getTruePose();
		auto gps = robot::readGPS();
		auto heading = robot::readIMUHeading();
		if (gps && (!lastGPSTime || lastGPSTime.value() < gps.getTime())) {
			lastGPSTime = gps.getTime();
		} else {
			gps = {};
		}
		if (heading && (!lastHeadingTime || lastHeadingTime.value() < heading.getTime())) {
			lastHeadingTime = heading.getTime();
		} else {
			heading = {};
		}

		if (truePoseDP) {
			if (!lastTruthTime || lastTruthTime.value() < truePoseDP.getTime()) {
				lastTruthTime = truePoseDP.getTime();
			}
			pose_t truePose = truePoseDP.getData();
			truePoseStream << duration_cast<std::chrono::milliseconds>(truePoseDP.getTime() -
																	   startTime)
								  .count()
						   << "," << truePose(0) << "," << truePose(1) << "," << truePose(2)
						   << "\n";
		}
		for (auto filterType : filters) {
			if (filterType == FilterType::EKF) {
				if (gps) {
					fpe.correctGPS(gps.getData());
				}
				if (heading) {
					fpe.correctHeading(heading.getData());
				}
				pose_t pose = fpe.getPose();
				auto ms = duration_cast<std::chrono::milliseconds>(now - startTime).count();
				filterStreamMap.at(filterType)
					<< ms << "," << pose(0) << "," << pose(1) << "," << pose(2) << "\n";

				fpe.predict(thetaVel, xVel);
			} else {
				CHECK_F(false);
			}
		}
		std::this_thread::sleep_until(now + 100ms);
	}
	truePoseStream.close();
	for (auto& [filterType, stream] : filterStreamMap) {
		stream.close();
	}
	std::cout << "Done!" << std::endl;
	Globals::websocketServer.stop();
}

void usage(const char* name) {
	std::string nameStr = "FilterPerfTest";
	if (name != nullptr) {
		nameStr.assign(name);
	}
	std::cout << "Usage: " << name << " <output folder> <filter1> <filter2> ..." << std::endl;
}

template <typename K, typename V>
constexpr std::unordered_map<V, K> reverseMap(const std::unordered_map<K, V>& map) {
	std::unordered_map<V, K> ret;
	for (const auto& [key, val] : map) {
		ret.insert({val, key});
	}
	return ret;
}
