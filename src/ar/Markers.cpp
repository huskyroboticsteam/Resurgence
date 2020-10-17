#include "Markers.h"

#include <memory>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace AR
{
namespace Markers
{

/**
   2D array of the bits in the ALVAR markers, used in URC. Each marker is 5x5, not
   including border, which is 2 bits wide. 0 is black, 1 is white. The markers can be
   found here:

   https://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers0to8.png
   https://wiki.ros.org/ar_track_alvar?action=AttachFile&do=view&target=markers9to17.png

   Note this array is only used internally.
 */

constexpr size_t ALVAR_COUNT = 4;
constexpr size_t ALVAR_BIT_SIZE = 5;
constexpr size_t ALVAR_BORDER_SIZE = 2;
constexpr size_t ALVAR_PHYS_SIZE = 200;
constexpr size_t BIT_ARR_SIZE = ALVAR_BIT_SIZE * ALVAR_BIT_SIZE;

constexpr size_t ARUCO_BIT_SIZE = 4;
constexpr size_t ARUCO_BORDER_SIZE = 1;
constexpr size_t ARUCO_PHYS_SIZE = 100; // we don't know this yet, change later

uint8_t __alvar_markers[ALVAR_COUNT][BIT_ARR_SIZE] = {
	// we have to turn the formatter off here because it's easier to tell the shape of the
	// tags in this format.

	// clang-format off

	// ALVAR Marker 1
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 1,1,1,1,1,
	 1,1,1,1,1},
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
	 1,0,1,1,0,
	 1,0,1,1,0},
	// ALVAR Marker 3
	{1,1,0,1,1,
	 1,1,0,1,1,
	 1,0,1,0,1,
	 0,1,1,1,1,
	 1,0,1,0,0}
	// will add more markers later. this is just to get it started
	// clang-format on
};

cv::Mat makeAlvarBytesList(uint8_t marker_array[ALVAR_COUNT][BIT_ARR_SIZE])
{
	cv::Mat bytes_list;
	cv::Size size(ALVAR_BIT_SIZE, ALVAR_BIT_SIZE);
	for (size_t m = 0; m < ALVAR_COUNT; m++)
	{
		uint8_t *current_marker = marker_array[m];
		cv::Mat current_marker_mat = cv::Mat(size, CV_8UC1, current_marker);
		bytes_list.push_back(current_marker_mat);
	}
	return bytes_list;
}

cv::aruco::Dictionary alvar_dict(makeAlvarBytesList(__alvar_markers), ALVAR_BIT_SIZE, 1);

cv::aruco::Dictionary circ_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

MarkerSet<URCMarkerID> makeURCSet()
{
	MarkerSet<URCMarkerID> set(ALVAR_BIT_SIZE, ALVAR_PHYS_SIZE);
	set.addAllFromDict(alvar_dict);
	set.addIDMapping(0, LEG1);
	// TODO add all mappings
	return set;
}

MarkerSet<CIRCMarkerID> makeCIRCSet()
{
	MarkerSet<CIRCMarkerID> set(ARUCO_BIT_SIZE, ARUCO_PHYS_SIZE);
	set.addAllFromDict(circ_dict);
	set.addIDMapping(0, CIRCMarker1);
	// TODO add all mappings
	return set;
}

const std::shared_ptr<MarkerSet<URCMarkerID>> urc_set =
	std::make_shared<MarkerSet<URCMarkerID>>(makeURCSet());
const std::shared_ptr<MarkerSet<CIRCMarkerID>> circ_set =
	std::make_shared<MarkerSet<CIRCMarkerID>>(makeCIRCSet());

const std::shared_ptr<MarkerSet<URCMarkerID>> URC_MARKERS()
{
	return urc_set;
}

const std::shared_ptr<MarkerSet<CIRCMarkerID>> CIRC_MARKERS()
{
	return circ_set;
}

}; // namespace Markers
}; // namespace AR
