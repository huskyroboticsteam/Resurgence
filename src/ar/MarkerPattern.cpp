#include "MarkerPattern.h"

#include <cassert>
#include <cstdint>
#include <memory>

#include <opencv2/core.hpp>

using mat_ptr = cv::Ptr<cv::Mat>;

namespace AR {

///////// Marker class implementation ///////////////
MarkerPattern::MarkerPattern(uint8_t data_region_size, uint8_t border_size, cv::Mat bits,
							 int id)
	: data_region_size(data_region_size), border_size(border_size), id(id),
	  data_bits(cv::Ptr<cv::Mat>(new cv::Mat(bits))) {
	assert(data_region_size > 0);
	assert(border_size > 0);
	assert(bits.rows == data_region_size);
	assert(bits.cols == data_region_size);
}

MarkerPattern::MarkerPattern() {
}

bool MarkerPattern::empty() const {
	return this->data_bits.empty();
}

uint8_t MarkerPattern::getDataRegionSize() const {
	return this->data_region_size;
}

uint8_t MarkerPattern::getBorderSize() const {
	return this->border_size;
}

const mat_ptr MarkerPattern::getDataBits() const {
	return this->data_bits;
}

int MarkerPattern::getId() const {
	return this->id;
}

bool MarkerPattern::operator==(const MarkerPattern& other) const {
	return (this->data_region_size == other.data_region_size) &&
		   (this->border_size == other.border_size) && (this->id == other.id) &&
		   (std::equal(this->data_bits->begin<uint8_t>(), this->data_bits->end<uint8_t>(),
					   other.data_bits->begin<uint8_t>()));
}
} // namespace AR
