#include "Tag.h"

namespace AR
{
	double Tag::findAngle(cv::Point a, cv::Point b, cv::Point c) const
	{
		a = a - b;  // shift vectors so that B is at the origin
		b = b - b;
		c = c - b;
		double dot_product = a.ddot(c);
		double a_mag = sqrt((a.x * a.x) + (a.y * a.y));
		double c_mag = sqrt((c.x * c.x) + (c.y * c.y));
		return acos(dot_product / (a_mag * c_mag));
	}
}
