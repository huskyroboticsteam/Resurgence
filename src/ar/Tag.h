#pragma once

#include <opencv2/core/types.hpp>

#include <vector>

namespace AR
{
	struct Corner
	{
		float angle;
		cv::Point point;
	};

	enum CornerIndex{
		TOP_LEFT = 0,
		TOP_RIGHT= 1,
		BOTTOM_RIGHT = 2,
		BOTTOM_LEFT = 3
	};
	
	class Tag
	{
	private:
		std::vector<Corner> corners;
	public:
		Tag(cv::Point top_left,
			cv::Point top_right,
			cv::Point bottom_right,
			cv::Point bottom_left);
		cv::Point getCenter() const;
		float getPitch() const;
		float getYaw() const;
		float getDistance() const;
	};
}
