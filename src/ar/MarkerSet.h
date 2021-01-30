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

   A MarkerSet is constructed from an ARUco Dictionary, which contains a list of all defined
   marker patterns, which all have some internal ID; the first marker in the Dictionary has ID
   0, the second has ID 1, etc. These IDs are usually meaningless and only refer to the order
   of the markers in the dictionary. Therefore, the MarkerSet class introduces a concept of an
   ID mapping; that is, a mapping from a marker's actual ID (as defined by the ARUco
   dictionary) to a user-defined ID that may have some actual meaning in context. Not all
   markers in the dictionary have to be mapped; the user may choose to map only some meaningful
   ones and leave others without a mapping.

   Note that you should probably not need to construct any instances of this class yourself;
   Markers.h will contain predefined MarkerSets that you should use.
 */
class MarkerSet
{
private:
	cv::Ptr<cv::aruco::Dictionary> dict;
	std::vector<Marker> markers;
	float physical_size;
	uint8_t data_region_size;
	uint8_t border_size;
	std::unordered_map<int, int> id_mappings;
	void init(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::Ptr<cv::aruco::Dictionary> markerDict);

public:
	/**
	   Constructs a new marker set. Note that you should likely not need to do this as a client
	   of the AR tag detection system.

	   Parameters:
	   - data_region_size: The size of the data region on the marker in pixels/blocks
	   (NOT including the border).

	   - border_size: The size of the border on ONE side of the marker. For example, if the
	   total marker size is 9 pixels and the data region is only 5 pixels wide, then the border
	   size will be 2 (instead of 4).

	   - physical_size: The size (in real-world units) of the tags that the markers will appear
	   on. It is important that this is the same scale as the units used for camera calibration
	   or otherwise the estimated measurements will be off by factors of 10.

	   - markerDict: The ARUco dictionary containing marker patterns.
	 */
	MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::aruco::Dictionary markerDict);
	/**
	   Constructs a new marker set. Note that you should likely not need to do this as a client
	   of the AR tag detection system.

	   Parameters:
	   - data_region_size: The size of the data region on the marker in pixels/blocks
	   (NOT including the border).

	   - border_size: The size of the border on ONE side of the marker. For example, if the
	   total marker size is 9 pixels and the data region is only 5 pixels wide, then the border
	   size will be 2 (instead of 4).

	   - physical_size: The size (in real-world units) of the tags that the markers will appear
	   on. It is important that this is the same scale as the units used for camera calibration
	   or otherwise the estimated measurements will be off by factors of 10.

	   - markerDictPtr: An OpenCV shared pointer (cv::Ptr) to an ARUCO dictionary containing
		 marker patterns.
	 */
	MarkerSet(uint8_t data_region_size, uint8_t border_size, float physical_size,
			  cv::Ptr<cv::aruco::Dictionary> markerDictPtr);
	/**
	   Adds an ID mapping; that is, a mapping from a marker's actual ID (as defined by the
	   ARUco dictionary) to a user-defined ID that may have some actual meaning in context.
	 */
	bool addIDMapping(int id, int mapping);
	/**
	   Gets an ID mapping. The ID mapping should actually be defined; this method will throw a
	   std::out_of_range exception if the ID mapping does not exist. See isIDMapped().
	 */
	int getIDMapping(int id);
	/**
	   Gets an ID mapping, statically cast to the type given in the template type
	   parameter. The type given should be a type that an integer can be statically cast to
	   (e.g. enums). This method will throw a std::out_of_range exception if the ID mapping
	   does not exist. See isIDMapped().
	 */
	template <class IDMapping_t> IDMapping_t getIDMapping(int id);
	/**
	   Returns the underlying ARUco dictionary defining the marker patterns.
	 */
	cv::Ptr<cv::aruco::Dictionary> getDict() const;
	/**
	   Returns the data region size; i.e. the size of the data region of the marker (NOT
	   including border).
	 */
	uint8_t getDataRegionSize() const;
	/**
	   Returns the border size on ONE side of the marker. For example if the marker is 9 pixels
	   wide and the data region is 5 pixels wide, the border size will be 2 (since the border
	   is on both sides of the marker).
	 */
	uint8_t getBorderSize() const;
	/**
	   Returns the physical size (in real-world units) of the tags that the markers will appear
	   on. The unit scale is defined by the user when an instance of this class is
	   constructed.
	 */
	float getPhysicalSize() const;
	/**
	   Returns a vector of the Markers in this marker set.
	 */
	std::vector<Marker> getMarkers() const;
	/**
	   Gets a Marker by its ID, as defined by the ARUco Dictionary. If the Marker exists, this
	   method will return true and the Marker will be returned through the output parameter; if
	   it does not exist, this method will return false and the output parameter will not be
	   modified.
	 */
	bool getMarkerByID(int id, Marker &out) const;
	/**
	   Gets a Marker by its mapped ID (as defined with addIDMapping()). If the mapping exists,
	   this method will return true and the Marker will be returned through the output
	   parameter; if it does not exist, this method will return false and the output parameter
	   will not be modified.
	*/
	bool getMarkerByMappedID(int id, Marker &out) const;
	/**
	   Returns true if the given user-defined ID mapping exists and false if it does not.
	 */
	bool isIDMapped(int id) const;
	/**
	   Operator to add or get ID mappings; can be used instead of addIDMapping/getIDMapping.
	 */
	int &operator[](int id);
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
const std::shared_ptr<MarkerSet> URC_MARKERS();

/**
   Returns the set of markers that will be used in CIRC.
*/
const std::shared_ptr<MarkerSet> CIRC_MARKERS();

} // namespace Markers

} // namespace AR
