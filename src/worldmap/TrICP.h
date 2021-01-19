#pragma once
#include "GlobalMap.h"

struct PointPair
{
	point_t mapPoint;
	point_t samplePoint;
	double dist;
};

class TrICP
{
public:
	explicit TrICP(const GlobalMap &map, double overlap, int maxIter);
	points_t correct(const points_t &sample);

private:
	const GlobalMap &map;
	double overlap;
	int maxIter;
	transform_t computeTransformation(const std::vector<PointPair> &pairs);
	points_t iterate(const points_t &sample, int N, double &S);
	bool isDone(int numIter, double S);
};
