#include "TrICP.h"

#include <Eigen/LU>
#include <Eigen/SVD>

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

TrICP::TrICP(double overlap, int maxIter, double relErrChangeThresh)
	: overlap(overlap), maxIter(maxIter), relErrChangeThresh(relErrChangeThresh)
{
}

points_t TrICP::correct(const points_t &sample,
						const std::function<point_t(const point_t &)> &getClosest)
{
	if (sample.empty())
	{
		return sample;
	}
	int i = 0;
	double mse = 1e9;
	double oldMSE;
	points_t points = sample;
	int N = static_cast<int>(overlap * sample.size());
	do
	{
		i++;
		oldMSE = mse;
		points = iterate(points, N, getClosest, mse);
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

points_t TrICP::iterate(const points_t &sample, int N,
						const std::function<point_t(const point_t &)> &getClosest,
						double &mse) const
{
	PointPair pairs[sample.size()];
	for (int i = 0; i < sample.size(); i++)
	{
		const point_t &point = sample[i];
		point_t closestPoint = getClosest(point);
		double dist = (point - closestPoint).norm();
		PointPair pair{closestPoint, point, dist};
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
	 * We can do this with the Kabsch algorithm.
	 * https://en.wikipedia.org/wiki/Kabsch_algorithm
	 */
	// TODO: Maybe transform the map to match the data sample?
	Eigen::Vector2d mapCentroid = Eigen::Vector2d::Zero();
	Eigen::Vector2d sampleCentroid = Eigen::Vector2d::Zero();

	for (const PointPair &pair : pairs)
	{
		mapCentroid += pair.mapPoint.topRows<2>();
		sampleCentroid += pair.samplePoint.topRows<2>();
	}

	mapCentroid /= pairs.size();
	sampleCentroid /= pairs.size();

	int size = pairs.size();
	Eigen::Matrix<double, Eigen::Dynamic, 2> P(size, 2);
	Eigen::Matrix<double, Eigen::Dynamic, 2> Q(size, 2);
	for (int i = 0; i < size; i++)
	{
		P.row(i) = pairs[i].samplePoint.topRows<2>() - sampleCentroid;
		Q.row(i) = pairs[i].mapPoint.topRows<2>() - mapCentroid;
	}

	Eigen::Matrix2d H = P.transpose() * Q;

	// computing SVD for square matrices, so no QR preconditioner needed
	Eigen::JacobiSVD<Eigen::Matrix2d, Eigen::NoQRPreconditioner> svd;
	svd.compute(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

	Eigen::Matrix2d U = svd.matrixU();
	Eigen::Matrix2d V = svd.matrixV();
	double d = (V * U.transpose()).determinant() > 0 ? 1 : -1;
	Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
	D.bottomRightCorner<1, 1>()(0, 0) = d;
	Eigen::Matrix2d R = V * D * U.transpose();

	Eigen::Vector2d translation = mapCentroid - R * sampleCentroid;

	transform_t trf = transform_t::Identity();
	trf.topLeftCorner<2, 2>() = R;
	trf.topRightCorner<2, 1>() = translation;
	return trf;
}
