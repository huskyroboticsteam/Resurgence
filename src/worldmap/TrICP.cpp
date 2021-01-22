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

	std::vector<PointPair> ret;
	for (int i = len - 1; i > len - 1 - minNum; i--)
	{
		ret.push_back(arr[0]);
		std::swap(arr[0], arr[i]);
		heapify(arr, i, 0);
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
	double mse = 1e9;
	double oldMSE;
	points_t points = sample;
	int N = std::min(static_cast<int>(map.getPoints().size()),
					 static_cast<int>(overlap * sample.size()));
	do
	{
		i++;
		oldMSE = mse;
		points = iterate(points, N, mse);
	} while (!isDone(i, mse, oldMSE));

	return points;
}

bool TrICP::isDone(int numIter, double mse, double oldMSE) const
{
	if (mse <= 1e-9)
	{
		return true;
	}
	double relErrChange = abs(mse - oldMSE) / mse;
	return numIter >= maxIter || relErrChange <= relErrChangeThresh;
}

points_t TrICP::iterate(const points_t &sample, int N, double &mse) const
{
	PointPair pairs[sample.size()];
	for (int i = 0; i < sample.size(); i++)
	{
		const point_t &point = sample[i];
		point_t closest = map.getClosest(point);
		double dist = (point - closest).norm();
		PointPair pair{closest, point, dist};
		pairs[i] = pair;
	}

	std::vector<PointPair> closestPairs = heapsort(pairs, sample.size(), N);

	double newS = 0;
	for (const PointPair &pair : closestPairs)
	{
		newS += pair.dist * pair.dist;
	}
	mse = newS / N;

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
	/*
	 * We need to find an affine transformation that maps points in the sample to points in
	 * the map. This transformation is restricted, as only rotations and translations are
	 * permitted. Shearing and scaling is invalid.
	 *
	 * Instead of calculating the affine transformation directly, we'll calculate the
	 * parameters directly. We have 3: translate x, translate y, rotate. This manifests as 4
	 * variables: cosTheta, sinTheta, dx, dy. (can't directly calculate theta)
	 *
	 * This results in 2 equations per pair of points.
	 */
	size_t size = 2 * pairs.size();
	Eigen::Matrix<double, Eigen::Dynamic, 4> A(size, 4);
	Eigen::Matrix<double, Eigen::Dynamic, 1> B(size, 1);
	for (int i = 0; i < pairs.size(); i++)
	{
		int index = 2 * i;
		point_t from = pairs[i].samplePoint;
		point_t to = pairs[i].mapPoint;
		// x' = cosTheta * x - sinTheta * y + dx;
		Eigen::RowVector4d row1 = {from(0), -from(1), 1, 0};
		// y' = cosTheta * y + sinTheta * x + dy;
		Eigen::RowVector4d row2 = {from(1), from(0), 0, 1};
		A.row(index) = row1;
		A.row(index + 1) = row2;
		// first equation is for x', second is for y'
		B.block<2, 1>(index, 0) = to.topRows<2>();
	}
	// Least-squares solution. This vector is [cosTheta, sinTheta, dx, dy]
	Eigen::Matrix<double, 4, 1> parameters = A.colPivHouseholderQr().solve(B);
	double cosThetaApprox = parameters(0);
	double sinThetaApprox = parameters(1);
	double deltaX = parameters(2);
	double deltaY = parameters(3);
	// atan2 1) reconciles the different thetas in cosTheta and sinTheta (due to approximation)
	// and 2) gets rid of any scale factors
	double theta = atan2(sinThetaApprox, cosThetaApprox);
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	transform_t trf;
	trf << cosTheta, -sinTheta, deltaX, sinTheta, cosTheta, deltaY, 0, 0, 1;
	return trf;
}
