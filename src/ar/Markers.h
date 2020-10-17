#pragma once

#include <opencv2/aruco.hpp>

#include <memory>

#include "MarkerSet.h"

namespace AR
{
namespace Markers
{

enum URCMarkerID
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

enum CIRCMarkerID
{
	// I know nothing about what the AR markers are for CIRC yet. Will update this when I get
	// some information about that. This is a placeholder for now.
	CIRCMarker1,
	CIRCMarker2
};

/**
   Returns set of markers that will be used in URC.
*/
const std::shared_ptr<MarkerSet<URCMarkerID>> URC_MARKERS();

/**
   Returns set of markers that will be used in CIRC.
*/
const std::shared_ptr<MarkerSet<CIRCMarkerID>> CIRC_MARKERS();

} // namespace Markers
} // namespace AR
