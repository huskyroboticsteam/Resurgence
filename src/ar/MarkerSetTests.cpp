#include <catch2/catch.hpp>

#include "MarkerSet.h"

#define TAG "[ar][marker_set]"
const cv::Ptr<cv::aruco::Dictionary> DICT_4X4_50 =
	cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

static AR::MarkerSet makeSet(){
	AR::MarkerSet ms = AR::MarkerSet(4, 1, 0.1, DICT_4X4_50);
	CHECK(ms.getDataRegionSize() == 4);
	CHECK(ms.getDataRegionSize() == DICT_4X4_50->markerSize);
	CHECK(ms.getBorderSize() == 1);
	CHECK(ms.getPhysicalSize() == Approx(0.1));
	return ms;
}



namespace AR
{

TEST_CASE("Constructing MarkerSet test", TAG)
{
	REQUIRE_NOTHROW(makeSet());
}

TEST_CASE("Predefined MarkerSets defined correctly", TAG)
{
	CHECK(Markers::URC_MARKERS());
	CHECK(Markers::CIRC_MARKERS());
}

TEST_CASE("Adding and retrieving ID mappings works", TAG)
{
	MarkerSet ms = makeSet();
	SECTION("Mappings that are added can be retrieved")
	{
		ms.addIDMapping(0, 0);
		ms.addIDMapping(1, 3);
		CHECK(ms.isIDMapped(0));
		CHECK(ms.isIDMapped(1));
		CHECK(ms.getIDMapping(0) == 0);
		CHECK(ms.getIDMapping(1) == 3);
	}
	SECTION("Retrieving non-existent mapping throws exception")
	{
		ms.addIDMapping(0, 0);
		ms.addIDMapping(1, 3);
		REQUIRE_THROWS_AS(ms.getIDMapping(2), std::out_of_range);
		REQUIRE_THROWS_AS(ms.getIDMapping(45), std::out_of_range);
	}
	SECTION("[] operator can be used to get/set mappings")
	{
		ms[4] = 5;
		ms[6] = -1;
		CHECK(ms[6] == -1);
		CHECK(ms[4] == 5);
	}
	
	SECTION("Mappings can be retrieved as a compatible type")
	{
		enum TestType
		{
			A,
			B,
			C,
			D
		};

		ms.addIDMapping(0, 0);
		ms.addIDMapping(1, 1);
		ms.addIDMapping(4, 2);
		ms.addIDMapping(6, 3);
		CHECK(ms.getIDMappingCast<TestType>(0) == TestType::A);
		CHECK(ms.getIDMappingCast<TestType>(1) == TestType::B);
		CHECK(ms.getIDMappingCast<TestType>(4) == TestType::C);
		CHECK(ms.getIDMappingCast<TestType>(6) == TestType::D);
	}
}

TEST_CASE("Markers can be retrieved from the MarkerSet")
{
	MarkerSet ms = makeSet();
	std::vector<Marker> markers = ms.getMarkers();
	// marker vector should have the same number of markers as are in the
	// dictionary
	CHECK(markers.size() == DICT_4X4_50->bytesList.rows);
	SECTION("Markers should be the same as in the dictionary")
	{
		for (int i = 0; i < markers.size(); i++)
		{
			cv::Mat dict_marker_bits =
				DICT_4X4_50->getBitsFromByteList(DICT_4X4_50->bytesList.row(i),
												 ms.getDataRegionSize());
			CHECK(std::equal(dict_marker_bits.begin<uint8_t>(),
							 dict_marker_bits.end<uint8_t>(),
							 markers[i].getDataBits()->begin<uint8_t>()));
			CHECK(markers[i].getBorderSize() == ms.getBorderSize());
			CHECK(markers[i].getDataRegionSize() == ms.getDataRegionSize());
		}
	}
	SECTION("Markers can be retrieved by ID")
	{
		for (int i = 0; i < markers.size(); i++)
		{
			Marker from_vector = markers[i];
			Marker from_set;
			CHECK(ms.getMarkerByID(i, from_set));
			CHECK(from_vector == from_set);
		}
	}
	SECTION("Markers can be retrieved by a mapped ID")
	{
		int nums[] = {0, 3, 7, 11, 13};
		for(int i = 0; i < (sizeof(nums) / sizeof(int)); i++){
			ms.addIDMapping(nums[i], i);
		}
		for(int i = 0; i < (sizeof(nums) / sizeof(int)); i++){
			Marker m;
			CHECK(ms.getMarkerByMappedID(i, m));
			CHECK(markers[nums[i]] == m);
		}
		// non-existent ID should return false
		Marker m;
		CHECK_FALSE(ms.getMarkerByMappedID(-1, m));
	}
}

} // namespace AR
