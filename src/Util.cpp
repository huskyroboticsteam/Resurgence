#include "Util.h"

#include <cmath>

namespace util
{
bool almostEqual(double a, double b, double threshold)
{
	return std::abs(a - b) < threshold;
}
} // namespace util
