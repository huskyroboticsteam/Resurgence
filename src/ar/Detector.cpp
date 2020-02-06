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
// Declares all functions that will be used
bool isTagData(cv::Mat &data);
cv::Mat readData(cv::Mat &input);
TagID getTagIDFromData(cv::Mat& data);
std::vector<int> getBitData(cv::Mat data);
cv::Mat removeNoise(cv::Mat input, int blur_size = 5);
int averageRegion(cv::Mat &input, int x1, int y1, int x2, int y2);
std::vector<cv::Point2f> sortCorners(std::vector<cv::Point2f> quad);
std::vector<std::vector<cv::Point2f> > getQuads(cv::Mat input, cv::Mat &edges, cv::Mat &grayscale,
                                            	int canny_thresh_1, int canny_thresh_2, int blur_size);
bool contourInsideAnother(std::vector<cv::Point2f> input, std::vector<cv::Point2f> other);
cv::Mat prepImage(cv::Mat input, int blur_size, int thresh_val, int thresh_val2,
                  cv::Mat &grayscale);
void approxContours(std::vector<std::vector<cv::Point> > contours,
					std::vector<std::vector<cv::Point2f> > &quads);


// Overloaded function, only used if grayscale image and outline of edges are not needed
std::vector<Tag> Detector::findTags(cv::Mat input, int canny_thresh_1, int canny_thresh_2, int blur_size)
{
	cv::Mat junk; // this Mat won't be used later. it's just used to be passed as a
	              // reference to the function this is overloading.
	return findTags(input, junk, junk, canny_thresh_1, canny_thresh_2, blur_size);
}

// Stores grayscale version and outlined edged version of input image to grayscale and edges
// Returns a vector of Tags obtained from the picture
std::vector<Tag> Detector::findTags(cv::Mat input, cv::Mat &grayscale, cv::Mat &edges,
                          int canny_thresh_1, int canny_thresh_2, int blur_size)
{
	// Stores all quadrilateral shapes found in the image to the vector: allQuads
	std::vector<std::vector<cv::Point2f> > allQuads 
		= getQuads(input, edges, grayscale, canny_thresh_1, canny_thresh_2, blur_size);

	// vector to hold quadrilaterals that are definitely tags 
	// ~ used to check for duplicates
	std::vector<std::vector<cv::Point2f> > tag_quads;

	// vector to hold Tag Data that will be returned
	std::vector<Tag> output;

	// set of "ideal" points used for perspective transform
	std::vector<cv::Point2f> ideal;
	ideal.push_back(cv::Point2f(0, 0));
	ideal.push_back(cv::Point2f(IDEAL_TAG_SIZE, 0));
	ideal.push_back(cv::Point2f(IDEAL_TAG_SIZE, IDEAL_TAG_SIZE));
	ideal.push_back(cv::Point2f(0, IDEAL_TAG_SIZE));

	// loop over all quadrilateral candidates
	for (size_t i = 0; i < allQuads.size(); i++)
	{
		// Sorts corner points in order of: top-left, top-right, 
		// bottom-right, bottom-left and stores it in quad
		std::vector<cv::Point2f> quad = sortCorners(allQuads[i]);

		// perspective transform the quadrilateral to a flat square
		cv::Mat transform = cv::getPerspectiveTransform(quad, ideal);
		cv::Mat square;
		cv::warpPerspective(grayscale, square, transform,
		                    cv::Size(IDEAL_TAG_SIZE, IDEAL_TAG_SIZE));

		// Attempt to read the data as if it was an AR tag
		cv::Mat data = readData(square);

		// Checks if data matches criteria for an AR tag
		if (isTagData(data))
		{
			// check if any corners of this quadrilateral are inside one already found (i.e. it
			// is a duplicate)
			bool duplicate = false;
			for (std::vector<cv::Point2f> q : tag_quads)
			{
				duplicate = duplicate || contourInsideAnother(quad, q);
			}
			// if not a duplicate:
			if (!duplicate)
			{
				// add it to tag quadrilaterals vector
				tag_quads.push_back(quad);
				// determine id
				TagID id = getTagIDFromData(data);
				// Create new Tag object with approximated coordinates
				Tag tag(quad[0], quad[1], quad[2], quad[3], cam, id);
				output.push_back(tag);
			}
		}
	}

	return output;
}

std::vector<std::vector<cv::Point2f> > getQuads(cv::Mat input, cv::Mat &edges, cv::Mat &grayscale,
                                                int canny_thresh_1, int canny_thresh_2, int blur_size)
{
    // Applies grayscale, outlines objects in the picture and emphasizes outlines
	edges = prepImage(input, blur_size, canny_thresh_1, canny_thresh_2, grayscale);

	// vector to hold any contours found
	std::vector<std::vector<cv::Point> > contours;
	// vector to hold quadrilaterals that could be possible tags
	std::vector<std::vector<cv::Point2f> > quads;
	std::vector<cv::Vec4i> heirarchy; // ?? What is this vector for

	// Find contours in the image and saves it to contours
	cv::findContours(edges, contours, heirarchy, cv::RETR_TREE,
	                 cv::CHAIN_APPROX_SIMPLE);

	// Approximates contours with polygons. 
	// Saves contour to quads if polygon is a quadrilateral
	approxContours(contours, quads);

    return quads;
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

cv::Mat removeNoise(cv::Mat input, int blur_size)
{
	cv::Mat blur;
	int bsize = (blur_size * 2) + 1;
	cv::GaussianBlur(input, blur, cv::Size(bsize, bsize), 0);
	// FIXME: Change to bilateral blur or some other faster method.
	return blur;
}

void approxContours(std::vector<std::vector<cv::Point> > contours,
                    std::vector<std::vector<cv::Point2f> > &quads) 
{
    for (size_t i = 0; i < contours.size(); i++) {
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
}

std::vector<cv::Point2f> sortCorners(std::vector<cv::Point2f> quad) {
    std::cout << "image points (before sorting): " << quad;
    // sort points into the correct order (top left, top right, bottom right,
    // bottom left)
    std::sort(quad.begin(), quad.end(),
                [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.y < pt2.y; });
    std::sort(quad.begin(), quad.begin() + 2,
                [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x < pt2.x; });
    std::sort(quad.begin() + 2, quad.end(),
                [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x > pt2.x; });
    std::cout << " image points (after sorting): " << quad << std::endl;

    return quad;
}

/*
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
			// divides avg pixel value by threshold. Black is stored as 0, White as 1
			int val = averageRegion(input, x1, y1, x2, y2) / BLACK_THRESH;
			output.at<uint8_t>(r, c) = val;
		}
	}
	return output;
}

/*
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

bool isTagData(cv::Mat &data)
{
	// check for 2-wide black border
	for (int x = 0; x < TAG_GRID_SIZE; x++)
	{
		for (int y = 0; y < TAG_GRID_SIZE; y++)
		{
			// skip the region in the middle of the tag, we only care about the border
			if ((x < 2 || x > 6) || (y < 2 || y > 6))
			{
				if (data.at<uint8_t>(y, x) == 1)
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
		if (data.at<uint8_t>(p) == 1)
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

/*
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

TagID getTagIDFromData(cv::Mat& data)
{
	TagID tags[] = {ID_LEG_1, ID_LEG_4L, ID_LEG_4R, ID_LEG_6L, ID_UNKNOWN, ID_UNKNOWN,
                    ID_LEG_7L, ID_LEG_7R, ID_LEG_5L, ID_LEG_5R, ID_LEG_2, ID_LEG_3, ID_LEG_6R};

    std::vector<int> bits = getBitData(data);

    // Adds all the hamming code data
    std::vector<std::vector<int>> table;
    std::vector<int> zero = {0, 0, 0, 0, 0, 0, 0, 0}; table.push_back(zero);
    std::vector<int> one = {1, 0, 0, 0, 0, 1, 1, 1}; table.push_back(one);
    std::vector<int> two = {1, 0, 0, 1, 1, 0, 0, 1}; table.push_back(two);
    std::vector<int> three = {0, 0, 0, 1, 1, 1, 1, 0}; table.push_back(three);
    std::vector<int> four = {1, 0, 1, 0, 1, 0, 1, 0}; table.push_back(four);
    std::vector<int> five = {0, 0, 1, 0, 1, 1, 0, 1}; table.push_back(five);
    std::vector<int> six = {0, 0, 1, 1, 0, 0, 1, 1}; table.push_back(six);
    std::vector<int> seven = {1, 0, 1, 1, 0, 1, 0, 0}; table.push_back(seven);
    std::vector<int> eight = {0, 1, 0, 0, 1, 0, 1, 1}; table.push_back(eight);
    std::vector<int> nine = {1, 1, 0, 0, 1, 1, 0, 0}; table.push_back(nine);
    std::vector<int> ten = {1, 1, 0, 1, 0, 0, 1, 0}; table.push_back(ten);
    std::vector<int> eleven = {0, 1, 0, 1, 0, 1, 0, 1}; table.push_back(eleven);
    std::vector<int> twelve = {1, 1, 1, 0, 0, 0, 0, 1}; table.push_back(twelve);

    int found = 0;
    int number;

    // Loops over the table and find the closest matching bit code
    for (int i = 0; i < table.size() && found < 2; i++) {
        int count = 0;
        for (int j = 0; j < bits.size(); j++) {
            if (table[i][j] != bits[j]) {
                count++;
            }
        }

        if (count == 0) {
            number = i;
            found = 2;
        } else if (count == 1) {
            if (found == 0) {
                number = i;
				found++;
            } else {
                return ID_UNKNOWN; // Error: Cannot find single matching Hamming code
            }
        }
    }

    return tags[number];
}

std::vector<int> getBitData(cv::Mat data) 
{
    std::vector<int> bits;
    for (int row = 5; row < 7; row++) 
    {
        for (int col = 2; col < 4; col++)
        {
            if (data.at<uint8_t>(row, col) == 0)
            {
                bits.push_back(1);
            }
            else
            {
                bits.push_back(0);
            }
        }
        for (int col = 5; col < 7; col++)
        {
            if (data.at<uint8_t>(row, col) == 0)
            {
                bits.push_back(1);
            }
            else
            {
                bits.push_back(0);
            }
        }
    }
    return bits;
}


Detector::Detector(CameraParams params) : cam(params)
{
	cam = params;
}

} // namespace AR
