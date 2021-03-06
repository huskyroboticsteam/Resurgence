#include "Marker.h"

#include <memory>
#include <opencv2/core.hpp>
#include <cstdint>
#include <cassert>

using mat_ptr = cv::Ptr<cv::Mat>;

///////// Marker class implementation ///////////////
Marker::Marker(uint8_t data_region_size, uint8_t border_size, cv::Mat bits, int id)
	: data_region_size(data_region_size), border_size(border_size), id(id),
	  data_bits(cv::Ptr<cv::Mat>(new cv::Mat(bits)))
{
	assert(data_region_size > 0);
	assert(border_size > 0);
	assert(bits.rows == data_region_size);
	assert(bits.cols == data_region_size);
}

Marker::Marker() {}

bool Marker::empty() const
{
	return this->data_bits.empty();
}

uint8_t Marker::getDataRegionSize() const
{
	return this->data_region_size;
}

uint8_t Marker::getBorderSize() const
{
	return this->border_size;
}

const mat_ptr Marker::getDataBits() const
{
	return this->data_bits;
}

int Marker::getId() const
{
	return this->id;
}

bool Marker::operator==(const Marker& other) const
{
	return (this->data_region_size == other.data_region_size)
		&& (this->border_size == other.border_size)
		&& (this->id == other.id)
		&& (std::equal(this->data_bits->begin<uint8_t>(),
					   this->data_bits->end<uint8_t>(),
					   other.data_bits->begin<uint8_t>()));
}
