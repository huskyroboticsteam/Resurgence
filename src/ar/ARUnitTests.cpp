#include "Detector.h"

#include <catch2/catch.hpp>
#ifdef AR_TESTS_WINDOWS
#include <opencv2/highgui.hpp>
#endif
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <bitset>
#include <cstdlib>
#include <ctime>

constexpr int TAG_SIZE = 225;
constexpr int TAG_GRID_SIZE = 9;
constexpr int BLACK = 0;
constexpr int WHITE = 255;
constexpr int MARGIN = 10;

const AR::CameraParams PARAMS = AR::Params::WEBCAM_720_PARAMS;
const cv::Size IMAGE_SIZE(1280, 720);

constexpr std::bitset<8> hamming_codes[16] = {
	std::bitset<8>(0b00000000),
	std::bitset<8>(0b11100001),
	std::bitset<8>(0b10011001),
	std::bitset<8>(0b01111000),
	std::bitset<8>(0b01010101),
	std::bitset<8>(0b10110100),
	std::bitset<8>(0b11001100),
	std::bitset<8>(0b00101101),
	std::bitset<8>(0b11010010),
	std::bitset<8>(0b00110011),
	std::bitset<8>(0b01001011),
	std::bitset<8>(0b10101010),
	std::bitset<8>(0b10000111),
	std::bitset<8>(0b01100110),
	std::bitset<8>(0b00011110),
	std::bitset<8>(0b11111111)
};

// NOTE: Replace this with a previous seed to duplicate the same results.
const unsigned int seed = 1587240118; //time(NULL);

namespace AR
{

cv::Mat generateTag(unsigned int data, int size = TAG_SIZE)
{
#ifdef AR_TESTS_WINDOWS
	cv::namedWindow("Tag");
#endif
	cv::Mat tag(size, size, CV_8UC3);
	int square_width = size / TAG_GRID_SIZE;

	tag.setTo(cv::Scalar::all(WHITE));

	for (size_t row = 0; row < TAG_GRID_SIZE; row++)
	{
		for (size_t col = 0; col < TAG_GRID_SIZE; col++)
		{
			cv::Point2i ul(col * square_width, row * square_width);
			cv::Point2i lr(((col+1) * square_width) -1, ((row+1)*square_width)-1);
			uint8_t color = WHITE;
			
			// fill in border squares
			if ((row < 2 || row > 6) || (col < 2 || col > 6))
			{
				color = BLACK;
			}
			// fill in vertical part of orientation marker
			else if ((row < 4 && row > 1) && (col == 4))
			{
				color = BLACK;
			}
			// fill in horizontal part of orientation marker
			else if ((row == 4) && (col == 3 || col == 5))
			{
				color = BLACK;
			}
			// fill in data
			else if ((row == 5 || row == 6) && (col > 1 && col < 7 && col != 4))
			{
				uint8_t bit_index = (row == 5 ? 0 : 4) + (col > 4 ? col - 3 : col - 2);
				uint8_t bit = (data >> bit_index) & 1;
				color = (bit == 1 ? BLACK : WHITE);
			}

			cv::rectangle(tag, ul, lr, cv::Scalar::all(color), cv::FILLED);
		}
	}
#ifdef AR_TESTS_WINDOWS
	cv::imshow("Tag", tag);
#endif
	return tag;
}

unsigned int hammingEncode(unsigned int data)
{
	uint8_t d1 = data & 1;
	uint8_t d2 = (data >> 1) & 1;
	uint8_t d3 = (data >> 2) & 1;
	uint8_t d4 = (data >> 3) & 1;

	uint8_t p1 = d1 ^ d2 ^ d4;
	uint8_t p2 = d1 ^ d3 ^ d4;
	uint8_t p3 = d2 ^ d3 ^ d4;
	uint8_t p4 = d1 ^ d2 ^ d3 ^ d4 ^ p1 ^ p2 ^ p3;

	uint8_t output = (p1 << 7) + (p2 << 6) + (d1 << 5) + (p3 << 4)
		+ (d2 << 3) + (d3 << 2) + (d4 << 1) + p4;

	return output;
}

TEST_CASE("Hamming encoder works", "[ar][util]")
{
	for(unsigned int i = 0; i < 16; i++)
	{
		std::bitset<8> expected = hamming_codes[i];
		SECTION("Encoding "+std::to_string(i)+" should produce "+expected.to_string())
		{
			unsigned int encoded = hammingEncode(i);
			REQUIRE(encoded == expected.to_ulong());
		}
	}
}

TEST_CASE("Images that do not contain tags are not detected", "[ar][detect]")
{
	// set up image and detector
	cv::Mat img(IMAGE_SIZE, CV_8UC3);
	Detector detector(PARAMS);
	
	SECTION("No tags should be detected in all black image")
	{
		img.setTo(cv::Scalar::all(BLACK));
		std::vector<Tag> tags = detector.findTags(img);
		// no tags should be returned
		REQUIRE(tags.size() == 0);
	}

	SECTION("No tags should be detected in all white image")
	{
		img.setTo(cv::Scalar::all(WHITE));
		std::vector<Tag> tags = detector.findTags(img);
		// no tags should be returned
		REQUIRE(tags.size() == 0);
	}
}

TEST_CASE("IDs are properly detected", "[ar][id]")
{
#ifdef AR_TESTS_WINDOWS
	cv::namedWindow("Image");
	cv::namedWindow("Gray");
	cv::namedWindow("Edges");
#endif
	
	INFO("Seed for AR tag tests: " + std::to_string(seed));
	
	// Create the image and detector
	cv::Mat image(IMAGE_SIZE, CV_8UC3);
#ifdef AR_TESTS_WINDOWS
	cv::Mat gray;
	cv::Mat edges;
#endif
	Detector detector(PARAMS);

	int xmax = IMAGE_SIZE.width - TAG_SIZE - (MARGIN*2);
	int ymax = IMAGE_SIZE.height - TAG_SIZE - (MARGIN*2);
	
	for(size_t i = 0; i < sizeof(ALL_TAGIDS)/sizeof(ALL_TAGIDS[0]); i++)
	{
		TagID current = ALL_TAGIDS[i];
		unsigned int id = static_cast<unsigned int>(current);
		SECTION("Tag " + std::to_string(current) + " is detected")
		{
			unsigned int data = hammingEncode(id);			
			cv::Mat tag = generateTag(data);

			int xpos = (rand() % xmax) + MARGIN;
			int ypos = (rand() % ymax) + MARGIN;
			image.setTo(cv::Scalar::all(WHITE));
			tag.copyTo(image(cv::Rect(xpos,ypos,tag.cols,tag.rows)));
#ifdef AR_TESTS_WINDOWS
			std::vector<std::vector<cv::Point2f>> corners;
			std::vector<Tag> tags = detector.findTags(image, gray, edges, corners);
#else
			std::vector<Tag> tags = detector.findTags(image);
#endif

#ifdef AR_TESTS_WINDOWS
			cv::imshow("Image", image);
			cv::imshow("Gray", gray);
			cv::imshow("Edges", edges);
			cv::waitKey(0);
#endif

			REQUIRE(tags.size() == 1);
			REQUIRE(tags[0].getID() == current);
		}
	}
}

} // namespace AR
