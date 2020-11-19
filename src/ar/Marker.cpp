#include "Marker.h"

#include <memory>
#include <opencv2/core.hpp>
#include <cstdint>
#include <cassert>

using mat_ptr = std::shared_ptr<cv::Mat>;

///////// Marker class implementation ///////////////
Marker::Marker(uint8_t data_region_size, uint8_t border_size, cv::Mat bits, int id)
	: data_region_size(data_region_size), border_size(border_size), id(id),
	  data_bits(std::shared_ptr<cv::Mat>(new cv::Mat(bits)))
{
	assert(data_region_size > 0);
	assert(border_size > 0);
	assert(bits.rows == data_region_size);
	assert(bits.cols == data_region_size);
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
