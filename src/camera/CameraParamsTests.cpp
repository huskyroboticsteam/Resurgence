#include <cstdio>
#include <string>
#include <vector>

#include <catch2/catch.hpp>

#include "CameraParams.h"

namespace cam
{

static bool matEqual(const cv::Mat &a, const cv::Mat &b, Approx approx = Approx(0))
{
	if (a.size() != b.size())
	{
		return false;
	}
	cv::Mat diff;
	cv::absdiff(a, b, diff);
	double max = 0;
	cv::minMaxIdx(diff, nullptr, &max);
	return max == approx;
}

static void checkEqual(const CameraParams &a, const CameraParams &b)
{
	REQUIRE(matEqual(a.getCameraMatrix(), b.getCameraMatrix()));
	REQUIRE(matEqual(a.getDistCoeff(), b.getDistCoeff()));
	REQUIRE(a.getImageSize() == b.getImageSize());
}

TEST_CASE("Validation in constructor works", "[camera][camera_params]")
{
	cv::Mat invalid_size_K = cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
	cv::Mat valid_size_K = cv::Mat::eye(cv::Size(3, 3), CV_64FC1);
	std::vector<int> valid_sizes = {4, 5, 8, 12, 14};

	for (int i = 1; i <= 14; i++)
	{
		cv::Mat D = cv::Mat::zeros(cv::Size(i, 1), CV_64FC1);
		if (std::find(valid_sizes.begin(), valid_sizes.end(), i) == valid_sizes.end())
		{
			REQUIRE_THROWS(CameraParams(valid_size_K, D));
		}
		else
		{
			REQUIRE_NOTHROW(CameraParams(valid_size_K, D));
			REQUIRE_THROWS(CameraParams(invalid_size_K, D));
		}
	}
	cv::Mat empty_K, empty_D;
	REQUIRE_THROWS(CameraParams(empty_K, empty_D));
	CameraParams empty;
	REQUIRE(empty.empty());
}

const std::string filename = "test_camera_params.yml";

TEST_CASE("Serialization works", "[camera][camera_params]")
{
	CameraParams params1(cv::Mat::eye(cv::Size(3, 3), CV_64FC1),
						 cv::Mat::zeros(cv::Size(5, 1), CV_64FC1));
	CameraParams params2((cv::Mat_<double>(3, 3) << 640, 0, 320, 0, 480, 240, 0, 0, 1),
						 cv::Mat::zeros(cv::Size(14, 1), CV_64FC1));
	CameraParams params3((cv::Mat_<double>(3, 3) << 1280, 0, 640, 0, 720, 360, 0, 0, 1),
						 cv::Mat::zeros(cv::Size(14, 1), CV_64FC1), cv::Size(1280, 720));
	cv::FileStorage fs_write(filename, cv::FileStorage::WRITE);
	fs_write << "params1" << params1;
	fs_write << "params2" << params2;
	fs_write << "params3" << params3;
	fs_write.release();

	cv::FileStorage fs_read(filename, cv::FileStorage::READ);
	CameraParams read1, read2, read3;
	fs_read["params1"] >> read1;
	fs_read["params2"] >> read2;
	fs_read["params3"] >> read3;
	fs_read.release();

	checkEqual(params1, read1);
	checkEqual(params2, read2);
	checkEqual(params3, read3);
	remove(filename.c_str());
}

} // namespace cam
