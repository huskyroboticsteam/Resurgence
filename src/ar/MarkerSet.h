#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include "Marker.h"

namespace AR
{


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
	cv::Ptr<cv::aruco::Dictionary> dict;
	std::vector<Marker> markers;
	float physical_size;
	uint8_t data_region_size;
	uint8_t border_size;
	std::unordered_map<int, IDMapping_t> id_mappings;
	void init(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::Ptr<cv::aruco::Dictionary> markerDict);

public:
	MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::aruco::Dictionary markerDict);
	MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::Ptr<cv::aruco::Dictionary> markerDictPtr);
	bool addIDMapping(int id, IDMapping_t mapping);
	cv::Ptr<cv::aruco::Dictionary> getDict() const;
	uint8_t getDataRegionSize() const;
	uint8_t getBorderSize() const;
	float getPhysicalSize() const;
	// TODO: add some way to get a marker by ID
	std::vector<Marker> getMarkers() const;
	bool isIDMapped(int id) const;
};

namespace Markers
{
/**
   Enum of marker names for URC.
 */
enum URCMarkerName
{
	LEG1,
	LEG2,
	LEG3,
	LEG4_L,
	LEG4_R,
	LEG5_L,
	LEG5_R,
	LEG6_L,
	LEG6_R,
	LEG7_L,
	LEG7_R
};

/**
   Enum of marker names for CIRC.

   NOTE: I currently have no information about what the markers will represent for CIRC and
   which marker IDs are important. Will update when I get more information about that. As of
   right now, all members of this enum are placeholders.
 */
enum CIRCMarkerName
{
	CIRCMarker1,
	CIRCMarker2
};

/**
   Returns the set of markers that will be used in URC.
*/
const std::shared_ptr<MarkerSet<URCMarkerName>> URC_MARKERS();

/**
   Returns the set of markers that will be used in CIRC.
*/
const std::shared_ptr<MarkerSet<CIRCMarkerName>> CIRC_MARKERS();

} // namespace Markers

} // namespace AR
