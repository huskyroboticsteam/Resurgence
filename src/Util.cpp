#include "Util.h"

#include <iostream>
#include <time.h>

#include <sys/time.h>

namespace util {
bool almostEqual(double a, double b, double threshold) {
	return std::abs(a - b) < threshold;
}

double quatToHeading(double qw, double qx, double qy, double qz) {
	Eigen::Quaterniond quat(qw, qx, qy, qz);
	return quatToHeading(quat);
}

double quatToHeading(Eigen::Quaterniond quat) {
	quat.normalize();
	Eigen::Matrix3d rotMat = quat.toRotationMatrix();
	Eigen::Vector3d transformedX = rotMat * Eigen::Vector3d::UnitX();
	// flatten to xy-plane
	transformedX(2) = 0;
	// recover heading
	double heading = std::atan2(transformedX(1), transformedX(0));
	return heading;
}

ScopedTimer::ScopedTimer(std::string name)
	: startTime(std::chrono::high_resolution_clock::now()), name(std::move(name)) {}

ScopedTimer::ScopedTimer() : ScopedTimer("") {}

ScopedTimer::~ScopedTimer() {
	if (!name.empty()) {
		auto now = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
		std::cout << "[" << name << "] ElapsedTime: " << elapsed.count() << "us\n";
	}
}

std::chrono::microseconds ScopedTimer::elapsedTime() const {
	auto now = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
	return elapsed;
}
} // namespace util

long getElapsedUsecs(const struct timeval& tp_start, const struct timeval& tp_end) {
	long elapsed =
		(tp_end.tv_sec - tp_start.tv_sec) * 1000 * 1000 + (tp_end.tv_usec - tp_start.tv_usec);
	return elapsed;
}

long getElapsedUsecs(const struct timeval& tp_start) {
	struct timeval tp_end;
	gettimeofday(&tp_end, NULL);
	return getElapsedUsecs(tp_start, tp_end);
}
