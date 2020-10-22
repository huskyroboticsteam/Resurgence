#include "Markers.h"

#include <memory>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

namespace AR
{
namespace Markers
{

// Constants - please note that these are only used internally.
/** The number of ALVAR markers stored in the data array. */
constexpr size_t ALVAR_COUNT = 4;
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
		bytes_list.push_back(current_marker_mat);
	}
	return bytes_list;
}

/**
   Constructs the URC marker set
 */
MarkerSet<URCMarkerName> makeURCSet()
{
	static const cv::aruco::Dictionary alvar_dict(makeAlvarBytesList(__alvar_markers), ALVAR_BIT_SIZE, 1);
	MarkerSet<URCMarkerName> set(ALVAR_BIT_SIZE, ALVAR_PHYS_SIZE, alvar_dict);
	set.addIDMapping(0, LEG1);
	// TODO add all mappings
	return set;
}

/**
   Constructs the CIRC marker set
 */
MarkerSet<CIRCMarkerName> makeCIRCSet()
{
	static const cv::Ptr<cv::aruco::Dictionary> circ_dict_ptr =
	cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	MarkerSet<CIRCMarkerName> set(ARUCO_BIT_SIZE, ARUCO_PHYS_SIZE, circ_dict_ptr);
	set.addIDMapping(0, CIRCMarker1);
	// TODO add all mappings
	return set;
}

const std::shared_ptr<MarkerSet<URCMarkerName>> URC_MARKERS()
{
	static const MarkerSet<URCMarkerName> URC_SET = makeURCSet();
	return std::make_shared<MarkerSet<URCMarkerName>>(URC_SET);
}

const std::shared_ptr<MarkerSet<CIRCMarkerName>> CIRC_MARKERS()
{
	static const MarkerSet<CIRCMarkerName> CIRC_SET = makeCIRCSet();
	return std::make_shared<MarkerSet<CIRCMarkerName>>(CIRC_SET);
}

}; // namespace Markers
}; // namespace AR
