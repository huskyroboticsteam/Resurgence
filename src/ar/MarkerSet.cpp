#include "MarkerSet.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "Marker.h"

using ar_dict = cv::aruco::Dictionary;
using ar_dict_ptr = cv::Ptr<cv::aruco::Dictionary>;
using mat_ptr = cv::Ptr<cv::Mat>;

namespace AR
{

///////// MarkerSet class implementation //////////////
MarkerSet::MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
					 ar_dict markerDict)
{
	ar_dict *dict_ = new ar_dict(markerDict);
	init(data_region_size, border_size, physical_size, ar_dict_ptr(dict_));
}

MarkerSet::MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
					 ar_dict_ptr markerDictPtr)
{
	assert(markerDictPtr);
	init(data_region_size, border_size, physical_size, markerDictPtr);
}

void MarkerSet::init(uint8_t data_region_size, uint8_t border_size, float physical_size,
					 ar_dict_ptr markerDict)
{
	assert(data_region_size > 0);
	assert(border_size > 0);
	assert(physical_size > 0);
	this->data_region_size = data_region_size;
	this->border_size = border_size;
	this->physical_size = physical_size;
	this->dict = markerDict;

	std::vector<Marker> markerVec;
	cv::Mat bytesList;
	dict->bytesList.copyTo(bytesList);
	for (size_t i = 0; i < bytesList.rows; i++)
	{
		cv::Mat row = bytesList.row(i);
		cv::Mat markerBits = ar_dict::getBitsFromByteList(row, data_region_size);
		Marker current(data_region_size, border_size, markerBits, i);
		markerVec.push_back(current);
	}
	this->markers = markerVec;
}

bool MarkerSet::addIDMapping(int id, int mapping)
{
	if (this->id_mappings.find(id) != this->id_mappings.end())
	{
		this->id_mappings[id] = mapping;
		return true;
	}
	else
	{
		return false;
	}
}

int MarkerSet::getIDMapping(int id)
{
	return this->id_mappings.at(id);
}

int &MarkerSet::operator[](int id)
{
	return this->id_mappings[id];
}

template <class IDMapping_t> IDMapping_t MarkerSet::getIDMapping(int id)
{
	return static_cast<IDMapping_t>(this->id_mappings.at(id));
}

ar_dict_ptr MarkerSet::getDict() const
{
	return this->dict;
}

uint8_t MarkerSet::getDataRegionSize() const
{
	return this->data_region_size;
}

uint8_t MarkerSet::getBorderSize() const
{
	return this->border_size;
}

float MarkerSet::getPhysicalSize() const
{
	return this->physical_size;
}

std::vector<Marker> MarkerSet::getMarkers() const
{
	return this->markers;
}

bool MarkerSet::isIDMapped(int id) const
{
	return this->id_mappings.count(id) == 1;
}

bool MarkerSet::getMarkerByID(int id, Marker &out) const
{
	if (id < 0 || id > markers.size())
	{
		return false;
	}
	out = markers[id];
	return true;
}

bool MarkerSet::getMarkerByMappedID(int id, Marker &out) const
{
	if (!isIDMapped(id))
	{
		return false;
	}
	out = markers[this->id_mappings.at(id)];
	return true;
}

namespace Markers
{

// Constants - please note that these are only used internally.
/** The number of ALVAR markers stored in the data array. */
constexpr size_t ALVAR_COUNT = 18;
/** The size in pixels of the data region of an ALVAR marker. */
constexpr size_t ALVAR_BIT_SIZE = 5;
/** The border size in pixels of one side of an ALVAR marker. */
constexpr size_t ALVAR_BORDER_SIZE = 2;
/** The physical size (in mm) of an ALVAR marker. */
constexpr size_t ALVAR_PHYS_SIZE = 200;
/** The bit array size for one ALVAR marker, equal to the bit size squared. */
constexpr size_t BIT_ARR_SIZE = ALVAR_BIT_SIZE * ALVAR_BIT_SIZE;

/** The size in pixels of the data region of an ARUco marker. */
constexpr size_t ARUCO_BIT_SIZE = 4;
/** The border size in pixels of one side of an ARUco marker. */
constexpr size_t ARUCO_BORDER_SIZE = 1;
/** The physical size (in mm) of an ARUco marker. */
constexpr size_t ARUCO_PHYS_SIZE = 100; // we don't know this yet, change later

/**
   2D array of the bits in the ALVAR markers, used in URC. Each marker is 5x5, not
   including border, which is 2 bits wide. 0 is black, 1 is white. The markers can be
   found here:

   https://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers0to8.png
   https://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers9to17.png

   Note this array is only used internally.
 */
uint8_t __alvar_markers[ALVAR_COUNT][BIT_ARR_SIZE] = {
	// we have to turn the formatter off here because it's easier to tell the shape of the
	// tags in this format.

	// clang-format off

	// ALVAR Marker 0
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 1,1,1,1,1,
	 1,1,1,1,1},
	// ALVAR Marker 1
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 0,0,1,1,0,
	 1,1,1,0,1},
	// ALVAR Marker 2
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 1,0,1,1,0,
	 1,0,1,1,0},
	// ALVAR Marker 3
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 0,1,1,1,1,
	 1,0,1,0,0},
	// ALVAR Marker 4
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 0,1,1,1,0,
	 0,1,1,1,0},
	// ALVAR Marker 5
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 1,0,1,1,1,
	 0,1,1,0,0},
	// ALVAR Marker 6
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 0,0,1,1,1,
	 0,0,1,1,1},
	// ALVAR Marker 7
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 1,1,1,1,0,
	 0,0,1,0,1},
	// ALVAR Marker 8
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 0,0,1,0,1,
	 1,1,1,1,0},
	// ALVAR Marker 9
	{1, 1, 0, 1, 1,
  	 1, 1, 0, 1, 1,
  	 1, 0, 1, 0, 1,
  	 1, 1, 1, 0, 0,
  	 1, 1, 1, 0, 0},
	// ALVAR Marker 10
 	{1, 1, 0, 1, 1,
  	 1, 1, 0, 1, 1,
  	 1, 0, 1, 0, 1,
  	 0, 1, 1, 0, 0,
  	 1, 0, 1, 1, 1},
	// ALVAR Marker 11
 	{1, 1, 0, 1, 1,
 	 1, 1, 0, 1, 1,
 	 1, 0, 1, 0, 1,
 	 1, 0, 1, 0, 1,
 	 1, 0, 1, 0, 1},
	// ALVAR Marker 12
 	{1, 1, 0, 1, 1,
 	 1, 1, 0, 1, 1,
  	 1, 0, 1, 0, 1,
 	 1, 0, 1, 0, 0,
 	 1, 1, 1, 1, 1},
	// ALVAR Marker 13
 	{1, 1, 0, 1, 1,
 	 1, 1, 0, 1, 1,
 	 1, 0, 1, 0, 1,
 	 0, 1, 1, 0, 1,
 	 0, 1, 1, 0, 1},
	// ALVAR Marker 14
 	{1, 1, 0, 1, 1,
 	 1, 1, 0, 1, 1,
 	 1, 0, 1, 0, 1,
 	 1, 1, 1, 0, 1,
 	 0, 0, 1, 1, 0},
	// ALVAR Marker 15
 	{1, 1, 0, 1, 1,
 	 1, 1, 0, 1, 1,
 	 1, 0, 1, 0, 1,
 	 0, 0, 1, 0, 0,
 	 0, 0, 1, 0, 0},
	// ALVAR Marker 16
 	{0, 0, 0, 1, 1,
 	 1, 1, 0, 0, 1,
 	 1, 0, 1, 0, 1,
 	 1, 1, 1, 1, 1,
 	 1, 1, 1, 1, 1},
	// ALVAR Marker 17
 	{0, 0, 0, 1, 1,
 	 1, 1, 0, 0, 1,
 	 1, 0, 1, 0, 1,
 	 0, 0, 1, 1, 0,
 	 1, 1, 1, 0, 1}

	// clang-format on
};

/**
   Constructs the "bytes list" (used by the ARUco module to construct dictionaries)
 */
cv::Mat makeAlvarBytesList(uint8_t marker_array[ALVAR_COUNT][BIT_ARR_SIZE])
{
	cv::Mat bytes_list;
	cv::Size size(ALVAR_BIT_SIZE, ALVAR_BIT_SIZE);
	for (size_t m = 0; m < ALVAR_COUNT; m++)
	{
		uint8_t *current_marker = marker_array[m];
		cv::Mat current_marker_mat = cv::Mat(size, CV_8UC1, current_marker);
		bytes_list.push_back(cv::aruco::Dictionary::getByteListFromBits(current_marker_mat));
	}
	return bytes_list;
}

/**
   Constructs the URC marker set
 */
MarkerSet makeURCSet()
{
	static const cv::aruco::Dictionary alvar_dict(makeAlvarBytesList(__alvar_markers),
												  ALVAR_BIT_SIZE, 1);
	MarkerSet set(ALVAR_BIT_SIZE, ALVAR_BORDER_SIZE, ALVAR_PHYS_SIZE, alvar_dict);

	set.addIDMapping(0, LEG1);
	set.addIDMapping(1, LEG2);
	set.addIDMapping(2, LEG3);
	set.addIDMapping(3, LEG4_L);
	set.addIDMapping(4, LEG4_R);
	set.addIDMapping(5, LEG5_L);
	set.addIDMapping(6, LEG5_R);
	set.addIDMapping(7, LEG6_L);
	set.addIDMapping(8, LEG6_R);
	set.addIDMapping(9, LEG7_L);
	set.addIDMapping(10, LEG7_R);

	return set;
}

/**
   Constructs the CIRC marker set
 */
MarkerSet makeCIRCSet()
{
	static const cv::Ptr<cv::aruco::Dictionary> circ_dict_ptr =
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	MarkerSet set(ARUCO_BIT_SIZE, ARUCO_BORDER_SIZE, ARUCO_PHYS_SIZE, circ_dict_ptr);
	set.addIDMapping(0, CIRCMarker1);
	// TODO: add all mappings
	return set;
}

const std::shared_ptr<MarkerSet> URC_MARKERS()
{
	static const MarkerSet URC_SET = makeURCSet();
	return std::make_shared<MarkerSet>(URC_SET);
}

const std::shared_ptr<MarkerSet> CIRC_MARKERS()
{
	static const MarkerSet CIRC_SET = makeCIRCSet();
	return std::make_shared<MarkerSet>(CIRC_SET);
}

}; // namespace Markers

} // namespace AR
