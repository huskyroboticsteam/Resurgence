#include "Detector.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

constexpr int CONTOUR_AREA_THRESH = 1000;
constexpr int TAG_GRID_SIZE = 9;
constexpr int IDEAL_TAG_SIZE = TAG_GRID_SIZE * 25;
constexpr int BLACK_THRESH = 150;

namespace AR
{
cv::Mat removeNoise(cv::Mat input, int blur_size = 5)
{
	cv::Mat blur;
	int bsize = (blur_size * 2) + 1;
	cv::GaussianBlur(input, blur, cv::Size(bsize, bsize), 0);
	// FIXME: Change to bilateral blur or some other faster method.
	return blur;
}

cv::Mat prepImage(cv::Mat input, int blur_size, int thresh_val, int thresh_val2,
                  cv::Mat &grayscale)
{
	cv::Mat blur = removeNoise(input, blur_size);
	cv::Mat gray;
	cv::Mat edges;

	cv::cvtColor(blur, gray, cv::COLOR_RGB2GRAY);
	cv::Canny(gray, edges, thresh_val, thresh_val2);
	cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1));

	grayscale = gray;

	return edges;
}

/**
   Averages the pixel values in the region of the given Mat bounded by the given coordinates.
*/
int averageRegion(cv::Mat &input, int x1, int y1, int x2, int y2)
{
	// get only the region specified
	cv::Mat region = input(cv::Rect2i(x1, y1, x2 - x1, y2 - y1));
	// average all pixels
	cv::Scalar mean = cv::mean(region);
	// get the first component of the scalar
	int avg = mean[0];
	return avg;
}

/**
   Reads the data in a given Mat containing a flat image of an AR Tag and returns it as a 9x9
   Mat of the average pixel values for each grid square.
 */
cv::Mat readData(cv::Mat &input)
{
	cv::Mat output = cv::Mat::zeros(TAG_GRID_SIZE, TAG_GRID_SIZE, CV_8UC1);
	int square_size = input.rows / TAG_GRID_SIZE;
	for (int r = 0; r < TAG_GRID_SIZE; r++)
	{
		for (int c = 0; c < TAG_GRID_SIZE; c++)
		{
			// average each region, and set the appropriate output value
			int x1 = c * square_size;
			int y1 = r * square_size;
			int x2 = (c + 1) * square_size;
			int y2 = (r + 1) * square_size;
			int val = averageRegion(input, x1, y1, x2, y2);
			output.at<uint8_t>(r, c) = val;
		}
	}
	return output;
}

bool isTagData(cv::Mat &data)
{
	// check for 2-wide black border
	for (int x = 0; x < TAG_GRID_SIZE; x++)
	{
		for (int y = 0; y < TAG_GRID_SIZE; y++)
		{
			// skip the region in the middle of the tag, we only care about the border
			if ((x < 2 || x > 6) && (y < 2 || y > 6))
			{
				if (data.at<uint8_t>(y, x) > BLACK_THRESH)
				{
					return false;
				}
			}
		}
	}

	// check for orientation shape
	cv::Point2i orientationPoints[] = {cv::Point2i(4, 2), cv::Point2i(2, 4), cv::Point2i(4, 6),
	                                   cv::Point2i(6, 4)};
	int whiteCount = 0;
	int blackCount = 0;
	for (cv::Point p : orientationPoints)
	{
		if (data.at<uint8_t>(p) > BLACK_THRESH)
		{
			whiteCount++;
		}
		else
		{
			blackCount++;
		}
	}
	// we should see 3 white squares and one black square in all the possible places for
	// orientation markers. If not, it's not a tag.
	return (whiteCount == 3 && blackCount == 1);
}

/**
   Checks whether or not any points of one contour are inside another contour. Used to filter
   out duplicates.
 */
bool contourInsideAnother(std::vector<cv::Point2f> input, std::vector<cv::Point2f> other)
{
	for (cv::Point p : input)
	{
		if (cv::pointPolygonTest(other, p, false) > 0)
		{
			return true;
		}
	}
	return false;
}

Detector::Detector(CameraParams params) : cam(params)
{
	cam = params;
}

std::vector<Tag> Detector::findTags(cv::Mat input, cv::Mat &grayscale, cv::Mat &edges,
                          int canny_thresh_1, int canny_thresh_2, int blur_size)
{
	// set of "ideal" points used for perspective transform
	std::vector<cv::Point2f> ideal;
	ideal.push_back(cv::Point2f(0, 0));
	ideal.push_back(cv::Point2f(IDEAL_TAG_SIZE, 0));
	ideal.push_back(cv::Point2f(IDEAL_TAG_SIZE, IDEAL_TAG_SIZE));
	ideal.push_back(cv::Point2f(0, IDEAL_TAG_SIZE));

	cv::Mat prepped_input =
	    prepImage(input, blur_size, canny_thresh_1, canny_thresh_2, grayscale);
	edges = prepped_input;

	// vector to hold the contours found
	std::vector<std::vector<cv::Point>> contours;
	// vector to hold quadrilaterals that could be possible tags
	std::vector<std::vector<cv::Point2f>> quads;
	// vector to hold quadrilaterals that are definitely tags
	std::vector<std::vector<cv::Point2f>> tag_quads;
	std::vector<cv::Vec4i> heirarchy;
	// vector to hold Tags for output
	std::vector<Tag> output;
	// find contours in the image
	cv::findContours(prepped_input, contours, heirarchy, cv::RETR_TREE,
	                 cv::CHAIN_APPROX_SIMPLE);

	// loop over all contours and approximate them with polygons
	for (size_t i = 0; i < contours.size(); i++)
	{
		std::vector<cv::Point2f> approx;
		cv::approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);
		// if polygons have 4 sides and are of an appropriately large area
		if (approx.size() == 4 && cv::isContourConvex(approx) &&
		    cv::contourArea(approx) > CONTOUR_AREA_THRESH)
		{
			// add them to quadrilateral candidates
			quads.push_back(approx);
		}
	}

	// loop over all quadrilateral candidates
	for (size_t i = 0; i < quads.size(); i++)
	{
		std::vector<cv::Point2f> current = quads[i];
		std::cout << "image points (before sorting): " << current;
		// sort points into the correct order (top left, top right, bottom right,
		// bottom left)
		std::sort(current.begin(), current.end(),
		          [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.y < pt2.y; });
		std::sort(current.begin(), current.begin() + 2,
		          [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x < pt2.x; });
		std::sort(current.begin() + 2, current.end(),
		          [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x > pt2.x; });
		std::cout << " image points (after sorting): " << current << std::endl;

		// perspective transform the quadrilateral to a flat square, and attempt to read data
		// from it like it is an AR tag
		cv::Mat transform = cv::getPerspectiveTransform(current, ideal);
		cv::Mat square;
		cv::warpPerspective(grayscale, square, transform,
		                    cv::Size(IDEAL_TAG_SIZE, IDEAL_TAG_SIZE));
		cv::Mat data = readData(square);

		// if the data read matches criteria for a tag
		if (isTagData(data))
		{
			// check if any corners of this quadrilateral are inside one already found (i.e. it
			// is a duplicate)
			bool duplicate = false;
			for (std::vector<cv::Point2f> q : tag_quads)
			{
				duplicate = contourInsideAnother(current, q) || duplicate;
			}
			// if not a duplicate:
			if (!duplicate)
			{
				// add it to tag quadrilaterals vector
				tag_quads.push_back(current);
				// make Tag with approximated coordinates
				Tag tag(current[0], current[1], current[2], current[3]);
				output.push_back(tag);
			}
		}
	}

	return output;
}

std::vector<Tag> Detector::findTags(cv::Mat input, int canny_thresh_1, int canny_thresh_2, int blur_size)
{
	cv::Mat junk; // this Mat won't be used later. it's just used to be passed as a
	              // reference to the function this is overloading.
	return findTags(input, junk, junk, canny_thresh_1, canny_thresh_2, blur_size);
}
} // namespace AR
