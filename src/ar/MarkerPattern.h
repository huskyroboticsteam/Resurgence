#pragma once

#include <cstdint>
#include <memory>

#include <opencv2/core.hpp>

namespace AR {

/**
   @addtogroup ar
   @{
 */

/**
   A class to represent the pattern that may appear on a physical AR tag, NOT a physical
   instance of a tag itself; each individual instance of the pattern as it appears in the world
   is represented by an instance of the Tag class, and there may be multiple Tags corresponding
   to the same MarkerPattern. MarkerPatterns are square patterns of either black or white
   pixels, usually with a border on the outside and a data region inside the
   border. MarkerPatterns should be unique and referred to uniquely by an ID, which is a
   non-negative integer that should be considered arbitrary and not guaranteed to have any
   actual meaning.

   You should likely not have to construct any instances of this class yourself; this will be
   done internally by the Detector class.
 */
class MarkerPattern {
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
	   Creates an empty marker, with a data region size, border size, and ID of 0, and an empty
	   matrix for the data bits. This is a default constructor just for convenience and you
	   should not try to use these empty markers. You can check if a marker is empty using the
	   Marker::empty() method.
	 */
	MarkerPattern();
	/**
	   Creates a marker, with the given data region size, border size, data bits, and id.
	 */
	MarkerPattern(uint8_t data_region_size, uint8_t border_size, cv::Mat bits, int id);
	bool empty() const;
	uint8_t getDataRegionSize() const;
	uint8_t getBorderSize() const;
	const cv::Ptr<cv::Mat> getDataBits() const;
	int getId() const;
	bool operator==(const MarkerPattern& other) const;
};

/** @} */

} // namespace AR
