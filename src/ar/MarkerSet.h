#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace AR
{
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
	cv::Mat data_bits;

public:
	/**
	   Creates a marker, with the given data region size, border size, data bits, and id.
	 */
	Marker(uint8_t data_region_size, uint8_t border_size, cv::Mat bits, int id);
	uint8_t getDataRegionSize() const;
	uint8_t getBorderSize() const;
	std::shared_ptr<cv::Mat> getDataBits() const;
	int getId() const;
};

/**
   Represents a "set" of markers used for a competition, containing the markers that should be
   recognized for that competition and also some other information like the physical
   (real-world) size of the tags. Additionally maps markers to some kind of descriptive meaning
   in the context for the competition; for example, a MarkerSet could be made for URC that
   contains all the marker patterns used by the ALVAR system, and maps certain marker patterns
   to enum constants describing which leg of the autonomous mission the markers represent.

   Note that you should probably not need to construct any instances of this class yourself;
   Markers.h will contain predefined MarkerSets that you should use.
 */
template <class IDMapping_t> class MarkerSet
{
private:
	cv::aruco::Dictionary dict;
	std::vector<Marker> markers;
	float physical_size;
	uint8_t data_region_size;
	uint8_t border_size;
	std::unordered_map<int, IDMapping_t> id_mappings;
	void init(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::aruco::Dictionary markerDict);

public:
	MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::aruco::Dictionary markerDict);
	MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::Ptr<cv::aruco::Dictionary> markerDictPtr);
	bool addIDMapping(int id, IDMapping_t mapping);
	cv::aruco::Dictionary getDict() const;
	uint8_t getDataRegionSize() const;
	uint8_t getBorderSize() const;
	float getPhysicalSize() const;
	std::vector<Marker> getMarkers() const;
	bool isIDMapped(int id) const;
};

} // namespace AR
