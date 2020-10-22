#pragma once

#include <opencv2/aruco.hpp>

#include <memory>

#include "MarkerSet.h"

namespace AR
{
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
