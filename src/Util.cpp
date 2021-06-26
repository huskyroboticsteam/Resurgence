#include "Util.h"

#include <iostream>
#include <time.h>
#include <sys/time.h>

namespace util
{
bool almostEqual(double a, double b, double threshold)
{
	return std::abs(a - b) < threshold;
}

ScopedTimer::ScopedTimer(std::string name)
	: name(std::move(name)), startTime(std::chrono::high_resolution_clock::now())
{
}

ScopedTimer::ScopedTimer() : ScopedTimer("")
{
}

ScopedTimer::~ScopedTimer()
{
	if (!name.empty())
	{
		auto now = std::chrono::high_resolution_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
		std::cout << "[" << name << "] ElapsedTime: " << elapsed.count() << "us\n";
	}
}

std::chrono::microseconds ScopedTimer::elapsedTime() const
{
	auto now = std::chrono::high_resolution_clock::now();
	auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime);
	return elapsed;
}
} // namespace util

long getElapsedUsecs(const struct timeval &tp_start, const struct timeval &tp_end) {
  long elapsed = (tp_end.tv_sec - tp_start.tv_sec) * 1000 * 1000 + (tp_end.tv_usec - tp_start.tv_usec);
  return elapsed;
}

long getElapsedUsecs(const struct timeval &tp_start) {
  struct timeval tp_end;
  gettimeofday(&tp_end, NULL);
  return getElapsedUsecs(tp_start, tp_end);
}
