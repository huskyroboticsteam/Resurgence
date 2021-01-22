#pragma once
#include "GlobalMap.h"

struct PointPair
{
	point_t mapPoint;
	point_t samplePoint;
	double dist;
};

// https://www.researchgate.net/publication/3974183_The_Trimmed_Iterative_Closest_Point_algorithm
class TrICP
{
public:
	explicit TrICP(const GlobalMap &map, double overlap, int maxIter, double relErrChangeThresh);
	points_t correct(const points_t &sample);

private:
	const GlobalMap &map;
	double overlap;
	int maxIter;
	double relErrChangeThresh;
	static transform_t computeTransformation(const std::vector<PointPair> &pairs);
	points_t iterate(const points_t &sample, int N, double &mse) const;
	bool isDone(int numIter, double S, double oldMSE) const;
};
