#include "TrICP.h"

#include <Eigen/QR>

void heapify(PointPair arr[], int len, int i)
{
	int largest = i;
	int l = 2 * i + 1;
	int r = r * i + 2;
	if (l < len && arr[i].dist > arr[largest].dist)
	{
		largest = l;
	}
	if (r < len && arr[i].dist < arr[largest].dist)
	{
		largest = r;
	}

	if (largest != i)
	{
		std::swap(arr[i], arr[largest]);
		heapify(arr, len, largest);
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
		struct PointPair pair
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
	// points is map, sample is new lidar sample (N samples that correspond)
	// M * A = B (A is sample, B is world map)
	// A^T * M^T = B^T, solve for M^T
	int size = pairs.size();
	Eigen::Matrix<double, Eigen::Dynamic, 3> A(size, 3);
	Eigen::Matrix<double, Eigen::Dynamic, 3> B(size, 3);
	for (int i = 0; i < pairs.size(); i++)
	{
		A.row(i) = pairs[i].samplePoint;
		B.row(i) = pairs[i].mapPoint;
	}
	Eigen::Matrix3d trfTrans = A.colPivHouseholderQr().solve(B);
	return trfTrans.transpose();
}
