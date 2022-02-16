#include "MarkerSet.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

using ar_dict = cv::aruco::Dictionary;
using ar_dict_ptr = cv::Ptr<cv::aruco::Dictionary>;
using mat_ptr = cv::Ptr<cv::Mat>;

namespace AR {

///////// MarkerSet class implementation //////////////
MarkerSet::MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
					 ar_dict markerDict) {
	ar_dict* dict_ = new ar_dict(markerDict);
	init(data_region_size, border_size, physical_size, ar_dict_ptr(dict_));
}

MarkerSet::MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
					 ar_dict_ptr markerDictPtr) {
	assert(markerDictPtr);
	init(data_region_size, border_size, physical_size, markerDictPtr);
}

void MarkerSet::init(uint8_t data_region_size, uint8_t border_size, float physical_size,
					 ar_dict_ptr markerDict) {
	assert(data_region_size > 0);
	assert(border_size > 0);
	assert(physical_size > 0);
	this->data_region_size = data_region_size;
	this->border_size = border_size;
	this->physical_size = physical_size;
	this->dict = markerDict;

	std::vector<MarkerPattern> markerVec;
	cv::Mat bytesList;
	dict->bytesList.copyTo(bytesList);
	for (int i = 0; i < bytesList.rows; i++) {
		cv::Mat row = bytesList.row(i);
		cv::Mat markerBits = ar_dict::getBitsFromByteList(row, data_region_size);
		MarkerPattern current(data_region_size, border_size, markerBits, i);
		markerVec.push_back(current);
	}
	this->markers = markerVec;
}

void MarkerSet::addIDMapping(int id, int mapping) {
	this->id_mappings[id] = mapping;
	this->reverse_mappings[mapping] = id;
}

int MarkerSet::getIDMapping(int id) const {
	return this->id_mappings.at(id);
}

int& MarkerSet::operator[](int id) {
	return this->id_mappings[id];
}

ar_dict_ptr MarkerSet::getDict() const {
	return this->dict;
}

uint8_t MarkerSet::getDataRegionSize() const {
	return this->data_region_size;
}

uint8_t MarkerSet::getBorderSize() const {
	return this->border_size;
}

float MarkerSet::getPhysicalSize() const {
	return this->physical_size;
}

std::vector<MarkerPattern> MarkerSet::getMarkers() const {
	return this->markers;
}

bool MarkerSet::isIDMapped(int id) const {
	return this->id_mappings.count(id) == 1;
}

bool MarkerSet::getMarkerByID(int id, MarkerPattern& out) const {
	if (id < 0 || static_cast<unsigned>(id) > markers.size()) {
		return false;
	}
	out = markers[id];
	return true;
}

bool MarkerSet::getMarkerByMappedID(int mapped_id, MarkerPattern& out) const {
	try {
		out = markers[this->reverse_mappings.at(mapped_id)];
		return true;
	} catch (std::out_of_range&) {
		return false;
	}
}

namespace Markers {

// Constants - please note that these are only used internally.
/** The size in pixels of the data region of an ARUco marker. */
constexpr size_t ARUCO_BIT_SIZE = 4;
/** The border size in pixels of one side of an ARUco marker. */
constexpr size_t ARUCO_BORDER_SIZE = 1;
/** The physical size (in mm) of an ARUco marker. */
constexpr float ARUCO_PHYS_SIZE = 0.15; // we don't know this yet, change later


/**
   Constructs the URC marker set
 */
MarkerSet makeURCSet() {
	static const cv::Ptr<cv::aruco::Dictionary> urc_dict_ptr =
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	MarkerSet set(ARUCO_BIT_SIZE, ARUCO_BORDER_SIZE, ARUCO_PHYS_SIZE, urc_dict_ptr);
	set.addIDMapping(0, START);
	set.addIDMapping(1, POST1);
	set.addIDMapping(2, POST2);
	set.addIDMapping(3, POST3);
	set.addIDMapping(4, GATEL);
	set.addIDMapping(5, GATER);

	return set;
}

/**
   Constructs the CIRC marker set
 */
MarkerSet makeCIRCSet() {
	static const cv::Ptr<cv::aruco::Dictionary> circ_dict_ptr =
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	MarkerSet set(ARUCO_BIT_SIZE, ARUCO_BORDER_SIZE, ARUCO_PHYS_SIZE, circ_dict_ptr);
	set.addIDMapping(0, CIRCMarker1);
	// TODO: add all mappings when we figure out what the CIRC markers are
	return set;
}

const std::shared_ptr<MarkerSet> URC_MARKERS() {
	static const MarkerSet URC_SET = makeURCSet();
	return std::make_shared<MarkerSet>(URC_SET);
}

const std::shared_ptr<MarkerSet> CIRC_MARKERS() {
	static const MarkerSet CIRC_SET = makeCIRCSet();
	return std::make_shared<MarkerSet>(CIRC_SET);
}

}; // namespace Markers

} // namespace AR
