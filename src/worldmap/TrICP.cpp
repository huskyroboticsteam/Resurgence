#include "TrICP.h"

#include <Eigen/QR>

void heapify(PointPair arr[], int len, int i)
{
	int smallest = i;
	int l = 2 * i + 1;
	int r = 2 * i + 2;
	if (l < len && arr[l].dist < arr[smallest].dist)
	{
		smallest = l;
	}
	if (r < len && arr[r].dist < arr[smallest].dist)
	{
		smallest = r;
	}

	if (smallest != i)
	{
		std::swap(arr[i], arr[smallest]);
		heapify(arr, len, smallest);
	}
}

std::vector<PointPair> heapsort(PointPair arr[], int len, int minNum)
{
	for (int i = len / 2 - 1; i >= 0; i--)
	{
		heapify(arr, len, i);
	}

	for (int i = len - 1; i > len - 1 - minNum; i--)
	{
		std::swap(arr[0], arr[i]);
		heapify(arr, i, 0);
	}

	std::vector<PointPair> ret;
	for (int i = len - 1; i > len - 1 - minNum; i--)
	{
		ret.push_back(arr[i]);
	}

	return ret;
}

TrICP::TrICP(const GlobalMap &map, double overlap, int maxIter, double relErrChangeThresh)
	: map(map), overlap(overlap), maxIter(maxIter), relErrChangeThresh(relErrChangeThresh)
{
}

points_t TrICP::correct(const points_t &sample)
{
	if (sample.empty() || map.getPoints().empty())
	{
		return sample;
	}
	int i = 0;
	double S = 1e9;
	double oldS;
	points_t points = sample;
	int N = static_cast<int>(overlap * sample.size());
	do {
		i++;
		oldS = S;
		points = iterate(points, N, S);
	} while (!isDone(i, S, oldS, N));

	return points;
}

bool TrICP::isDone(int numIter, double S, double oldS, int N) const
{
	double err = S / N;
	double prevErr = oldS / N;
	if (err <= 1e-9) {
		return true;
	}
	double relErrChange = abs(err - prevErr) / err;
	return numIter >= maxIter || relErrChange <= relErrChangeThresh;
}

points_t TrICP::iterate(const points_t &sample, int N, double &S) const
{
	PointPair pairs[sample.size()];
	for (int i = 0; i < sample.size(); i++)
	{
		const point_t &point = sample[i];
		point_t closest = map.getClosest(point);
		double dist = (point - closest).norm();
		PointPair pair
		{
			closest, point, dist
		};
		pairs[i] = pair;
	}

	std::vector<PointPair> closestPairs = heapsort(pairs, sample.size(), N);

	double newS = 0;
	for (const PointPair &pair : closestPairs)
	{
		newS += pair.dist * pair.dist;
	}
	S = newS;

	transform_t trf = computeTransformation(closestPairs);

	points_t ret;
	for (const point_t &point : sample)
	{
		ret.push_back(trf * point);
	}
	return ret;
}

transform_t TrICP::computeTransformation(const std::vector<PointPair> &pairs)
{
	// TODO: I think this is wrong. Adjust to disallow shear/scaling
	// points is map, sample is new lidar sample (N samples that correspond)
	// M * A = B (A is sample, B is world map)
	// A^T * M^T = B^T, solve for M^T
	int size = pairs.size();
	Eigen::Matrix<double, Eigen::Dynamic, 3> A(size, 3);
	Eigen::Matrix<double, Eigen::Dynamic, 2> B(size, 2);
	for (int i = 0; i < pairs.size(); i++)
	{
		A.row(i) = pairs[i].samplePoint;
		Eigen::Vector2d v = {pairs[i].mapPoint(0), pairs[i].mapPoint(1)};
		B.row(i) = v;//pairs[i].mapPoint.topRows<2>();
	}
	Eigen::Matrix<double, 3, 2> trfTrans = A.colPivHouseholderQr().solve(B);
	transform_t trf = transform_t::Identity();
	trf.block<2,3>(0,0) = trfTrans.transpose();
	return trf;
}
