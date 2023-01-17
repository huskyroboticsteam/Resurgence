#include "ScopedTimer.h"

#include <iostream>

namespace util {

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
