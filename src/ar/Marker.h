#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <cstdint>

/**
   A class to represent the pattern that may appear on a physical AR tag, NOT a physical
   instance of a tag itself; those are represented by the Tag class. Markers are square
   patterns of either black or white pixels, usually with a border on the outside and a data
   region inside the border. Markers should be unique and referred to by an ID, which may or
   may not be actually encoded in their data.

   You should likely not have to construct any instances of this class yourself.
 */
class Marker
{
private:
	/**
	   The width in pixels of the data region of each marker. Does not include the border.
	 */
	uint8_t data_region_size;
	/**
	   The width in pixels of the border of the marker. Note that this is only for ONE side of
	   the square; if the border is 1 pixel wide, then the marker's total size will be the data
	   region size + 2, for each border.
	 */
	uint8_t border_size;
	/**
	   The ID of the marker. Note that this need not actually be encoded in the data of the
	   marker, just a unique ID to distinguish this marker from the others.
	 */
	int id;
	/**
	   The actual bits stored in the data region of the marker, NOT including the border.
	 */
	cv::Ptr<cv::Mat> data_bits;

public:
	/**
	   Creates a marker, with the given data region size, border size, data bits, and id.
	 */
	Marker(uint8_t data_region_size, uint8_t border_size, cv::Mat bits, int id);
	uint8_t getDataRegionSize() const;
	uint8_t getBorderSize() const;
	const cv::Ptr<cv::Mat> getDataBits() const;
	int getId() const;
};
