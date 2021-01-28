#pragma once

#include "../simulator/utils.h"

// https://www.researchgate.net/publication/3974183_The_Trimmed_Iterative_Closest_Point_algorithm
class TrICP
{
public:
	explicit TrICP(int maxIter, double relErrChangeThresh,
				   std::function<point_t(const point_t &)> getClosest);
	transform_t correct(const points_t &sample, double overlap);

private:
	int maxIter;
	double relErrChangeThresh;
	std::function<point_t(const point_t &)> getClosest;
	transform_t iterate(points_t &sample, int N, double &mse) const;
	bool isDone(int numIter, double S, double oldMSE) const;
};
