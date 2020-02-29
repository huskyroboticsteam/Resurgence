#include "Detector.h"

#ifdef WITH_GPU
#include <opencv2/cudawarping.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/photo/cuda.hpp>
#endif
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <ctime>
#include <iostream>

constexpr int CONTOUR_AREA_THRESH = 1000;
constexpr int TAG_GRID_SIZE = 9;
constexpr int IDEAL_TAG_SIZE = TAG_GRID_SIZE * 25;
constexpr int BLACK_THRESH = 150;

constexpr int table[][8] = {{0, 0, 0, 0, 0, 0, 0, 0},
							{1, 0, 0, 0, 0, 1, 1, 1},
							{1, 0, 0, 1, 1, 0, 0, 1},
							{0, 0, 0, 1, 1, 1, 1, 0},
							{1, 0, 1, 0, 1, 0, 1, 0},
							{0, 0, 1, 0, 1, 1, 0, 1},
							{0, 0, 1, 1, 0, 0, 1, 1},
							{1, 0, 1, 1, 0, 1, 0, 0},
							{0, 1, 0, 0, 1, 0, 1, 1},
							{1, 1, 0, 0, 1, 1, 0, 0},
							{1, 1, 0, 1, 0, 0, 1, 0},
							{0, 1, 0, 1, 0, 1, 0, 1},
							{1, 1, 1, 0, 0, 0, 0, 1}};

namespace AR
{
// Declares all functions that will be used
bool readCheckData(cv::Mat &input, cv::Mat &output);
TagID getTagIDFromData(cv::Mat& data);
std::vector<int> getBitData(cv::Mat data);
std::vector<cv::Point2f> sortCorners(std::vector<cv::Point2f> quad);
std::vector<std::vector<cv::Point2f> > getQuads(cv::Mat input, cv::Mat &edges, cv::Mat &grayscale,
                                            	int canny_thresh_1, int canny_thresh_2, int blur_size);
bool contourInsideAnother(std::vector<cv::Point2f> input, std::vector<cv::Point2f> other);
cv::Mat prepImage(cv::Mat input, int thresh_val, int thresh_val2,
                  cv::Mat &grayscale, int blur_size = 5);
void approxContours(std::vector<std::vector<cv::Point> > contours,
					std::vector<std::vector<cv::Point2f> > &quads);


// Overloaded function, only used if grayscale image and outline of edges are not needed
std::vector<Tag> Detector::findTags(cv::Mat input, int canny_thresh_1, int canny_thresh_2, int blur_size)
{
	cv::Mat junk; // this Mat won't be used later. it's just used to be passed as a
	              // reference to the function this is overloading.
	std::vector<std::vector<cv::Point2f>> junk2;
	return findTags(input, junk, junk, junk2, canny_thresh_1, canny_thresh_2, blur_size);
}

// Stores grayscale version and outlined edged version of input image to grayscale and edges
// Returns a vector of Tags obtained from the picture
std::vector<Tag> Detector::findTags(cv::Mat input, cv::Mat &grayscale, cv::Mat &edges,
                          std::vector<std::vector<cv::Point2f> > &quad_corners,
						  int canny_thresh_1, int canny_thresh_2, int blur_size)
{

	std::clock_t c_start = std::clock(); // Stores current cpu time

	// Stores all quadrilateral shapes found in the image to the vector: allQuads
	std::vector<std::vector<cv::Point2f> > allQuads 
		= getQuads(input, edges, grayscale, canny_thresh_1, canny_thresh_2, blur_size);
	#ifdef WITH_GPU
		cv::cuda::GpuMat gpu_grayscale;
		gpu_grayscale.upload(grayscale);
	#endif
	std::cout << "Number of Quads found: " << allQuads.size() << std::endl;

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

		quad_corners.push_back(quad);

		// perspective transform the quadrilateral to a flat square
		cv::Mat transform = cv::getPerspectiveTransform(quad, ideal);
		cv::Mat square;
		#ifndef WITH_GPU
		cv::warpPerspective(grayscale, square, transform,
		                    cv::Size(IDEAL_TAG_SIZE, IDEAL_TAG_SIZE));
		#else
		cv::cuda::GpuMat gpu_square;
		cv::cuda::warpPerspective(gpu_grayscale, gpu_square, transform,
		                    cv::Size(IDEAL_TAG_SIZE, IDEAL_TAG_SIZE));
		gpu_square.download(square);
		#endif

		// Apply adaptive gaussian thresholding to the square, saves it to t_square
		cv::Mat t_square;
		cv::adaptiveThreshold(square, t_square, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 151, 0);

		// Create Mat to store tag data
		cv::Mat data = cv::Mat::zeros(TAG_GRID_SIZE, TAG_GRID_SIZE, CV_8UC1);

		// Reads the quadrilateral as if it was a tag
		// Returns true if it is a tag 
		// Tag data gets stored in data variable
		if (readCheckData(t_square, data))
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

	std::clock_t c_end = std::clock();

	long double time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
	std::cout << "Detector time used: " << time_elapsed_ms << " ms\n";

	return output;
}

std::vector<std::vector<cv::Point2f> > getQuads(cv::Mat input, cv::Mat &edges, cv::Mat &grayscale,
                                                int canny_thresh_1, int canny_thresh_2, int blur_size)
{
    // Applies grayscale, outlines objects in the picture and emphasizes outlines
	edges = prepImage(input, canny_thresh_1, canny_thresh_2, grayscale, blur_size);

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

cv::Mat prepImage(cv::Mat input, int thresh_val, int thresh_val2,
                  cv::Mat &grayscale, int blur_size)
{

	cv::Mat edges;
	#ifdef WITH_GPU
		// Declares cuda objects for edge detection, filtering, and dilating
		cv::Ptr<cv::cuda::CannyEdgeDetector> detector = 
			cv::cuda::createCannyEdgeDetector(50, 100);
		int bsize = blur_size * 2 + 1;
		cv::Ptr<cv::cuda::Filter> guassian_filter = 
			cv::cuda::createGaussianFilter(CV_8UC1, CV_8UC1, cv::Size(bsize, bsize), 0);
		cv::Ptr<cv::cuda::Filter> dilate_filter = 
			cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, cv::Mat());

		// Loads the input frame to a gpu equivalent
		cv::cuda::GpuMat gpu_input;
		gpu_input.upload(input);

		// Implements grayscale
		cv::cuda::GpuMat gpu_gray;
		cv::cuda::cvtColor(gpu_input, gpu_gray, cv::COLOR_BGR2GRAY);

		// Applies gaussian filter
		cv::cuda::GpuMat gpu_blur;
		guassian_filter->apply(gpu_gray, gpu_blur);

		// Detects and dilates the edges
		cv::cuda::GpuMat gpu_edges;
		detector->detect(gpu_blur, gpu_edges);
		dilate_filter->apply(gpu_edges, gpu_edges);

		// Converts the gpu matrices to normal matrices to be displayed
		gpu_gray.download(grayscale);
		gpu_edges.download(edges);
	#else
		cv::Mat gray;	
		cv::Mat blur;
		int bsize = (blur_size * 2) + 1;
		cv::GaussianBlur(input, blur, cv::Size(bsize, bsize), 0);
		cv::cvtColor(blur, gray, cv::COLOR_RGB2GRAY);
		cv::Canny(gray, edges, thresh_val, thresh_val2);
		cv::dilate(edges, edges, cv::Mat(), cv::Point(-1, -1));

		grayscale = gray;
	#endif

	return edges;
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
    //std::cout << "image points (before sorting): " << quad;
    // sort points into the correct order (top left, top right, bottom right,
    // bottom left)
    std::sort(quad.begin(), quad.end(),
                [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.y < pt2.y; });
    std::sort(quad.begin(), quad.begin() + 2,
                [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x < pt2.x; });
    std::sort(quad.begin() + 2, quad.end(),
                [](cv::Point pt1, cv::Point pt2) -> bool { return pt1.x > pt2.x; });
    //std::cout << " image points (after sorting): " << quad << std::endl;

    return quad;
}

bool readCheckData(cv::Mat &input, cv::Mat &output)
{
	// Create points to check for orientation
	cv::Point2i orientationPoints[] = {cv::Point2i(4, 2), cv::Point2i(2, 4), cv::Point2i(4, 6),
	                                   cv::Point2i(6, 4)};
	
	int square_size = input.rows / TAG_GRID_SIZE;
	int whiteCount = 0;
	int blackCount = 0;
	for (int row = 0; row < TAG_GRID_SIZE; row++)
	{
		for (int col = 0; col < TAG_GRID_SIZE; col++)
		{
			int x = col * square_size + (square_size / 2);
			int y = row * square_size + (square_size / 2);
			output.at<uint8_t>(row, col) = input.at<uint8_t>(y, x) / BLACK_THRESH;

			// Ensures that all borders are black
			if ((col < 2 || col > 6) || (row < 2 || row > 6)) {
				if (output.at<uint8_t>(row, col) == 1) {
					return false;
				}
			}
		}
	}

	// Checks for 3 white squares and 
	// 1 black square in orientation points.
	for (cv::Point p : orientationPoints)
	{
		if (output.at<uint8_t>(p) == 1)
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

    int found = 0;
    int bit = 4;

    // Loops over the table and find the closest matching bit code
    for (int i = 0; i < sizeof(table) && found < 2; i++) {
        int count = 0;
        for (int j = 0; j < bits.size(); j++) {
            if (table[i][j] != bits[j]) {
                count++;
            }
        }

		// Exits loop and sets the tag to the exact matching code
        if (count == 0) {
            bit = i;
            found = 2;

		// Sets the tag to the closest matching code
		// Only happens once, a second time will return ID_UNKNOWN
        } else if (count == 1) {
            if (found == 0) {
                bit = i;
				found++;
            } else {
                return ID_UNKNOWN; // Error: Cannot find single matching Hamming code
            }
        }
    }

    return tags[bit];
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
