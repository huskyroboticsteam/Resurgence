#include "MarkerSet.h"

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

using ar_dict = cv::aruco::Dictionary;
using ar_dict_ptr = cv::Ptr<cv::aruco::Dictionary>;
using mat_ptr = std::shared_ptr<cv::Mat>;

namespace AR
{
///////// Marker class implementation ///////////////
Marker::Marker(uint8_t data_region_size, uint8_t border_size, cv::Mat bits, int id)
{
	assert(data_region_size > 0);
	assert(border_size > 0);
	assert(bits.rows == data_region_size);
	assert(bits.cols == data_region_size);
	this->data_region_size = data_region_size;
	this->border_size = border_size;
	this->data_bits = bits;
	this->id = id;
}

uint8_t Marker::getDataRegionSize() const
{
	return this->data_region_size;
}

uint8_t Marker::getBorderSize() const
{
	return this->border_size;
}

mat_ptr Marker::getDataBits() const
{
	// we return data bits via shared pointer because it could be large.
	return std::make_shared<cv::Mat>(this->data_bits);
}

int Marker::getId() const
{
	return this->id;
}

///////// MarkerSet class implementation //////////////
template <class IDMapping_t>
MarkerSet<IDMapping_t>::MarkerSet(uint8_t data_region_size, uint8_t border_size,
								  float physical_size, ar_dict markerDict)
{
	init(data_region_size, border_size, physical_size, markerDict);
}

template <class IDMapping_t>
MarkerSet<IDMapping_t>::MarkerSet(uint8_t data_region_size, uint8_t border_size,
								  float physical_size, ar_dict_ptr markerDictPtr)
{
	assert(markerDictPtr);
	init(data_region_size, border_size, physical_size, *markerDictPtr);
}

template <class IDMapping_t>
void MarkerSet<IDMapping_t>::init(uint8_t data_region_size, uint8_t border_size,
								  float physical_size, ar_dict markerDict)
{
	assert(data_region_size > 0);
	assert(border_size > 0);
	assert(physical_size > 0);
	this->data_region_size = data_region_size;
	this->border_size = border_size;
	this->physical_size = physical_size;
	this->dict = markerDict;

	std::vector<Marker> markerVec;
	cv::Mat bytesList = dict.bytesList;
	for (size_t i = 0; i < bytesList.rows; i++)
	{
		cv::Mat row = bytesList.row(i);
		cv::Mat markerBits = ar_dict::getBitsFromByteList(row, data_region_size);
		Marker current(data_region_size, border_size, markerBits, i);
		markerVec.push_back(current);
	}
	this->markers = markerVec;
}

template <class IDMapping_t>
bool MarkerSet<IDMapping_t>::addIDMapping(int id, IDMapping_t mapping){
	if(id < this->markers.size()){
		this->id_mappings[id] = mapping;
	} else {
		return false;
	}
}

template <class IDMapping_t>
ar_dict MarkerSet<IDMapping_t>::getDict() const {
	return this->dict;
}

template <class IDMapping_t>
uint8_t MarkerSet<IDMapping_t>::getDataRegionSize() const {
	return this->data_region_size;
}

template <class IDMapping_t>
uint8_t MarkerSet<IDMapping_t>::getBorderSize() const {
	return this->border_size;
}

template <class IDMapping_t>
float MarkerSet<IDMapping_t>::getPhysicalSize() const {
	return this->physical_size;
}

template <class IDMapping_t>
std::vector<Marker> MarkerSet<IDMapping_t>::getMarkers() const {
	return this->markers;
}

template <class IDMapping_t>
bool MarkerSet<IDMapping_t>::isIDMapped(int id) const {
	return this->id_mappings.count(id) == 1;
}
} // namespace AR
