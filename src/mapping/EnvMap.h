#pragma once

#include <memory>
#include <vector>
#include <set>

constexpr float ChunkDimensionMeters = 1.0f;

// basic representation of an integer {x, y} pair
struct Vec2I {
    int x;
    int y;

    inline bool operator==(const Vec2I & other) {
        return (x == other.x && y == other.y);
    }
};

struct MapObstacle {
    // global position relative to start position
    float x, y;
    // the set of chunks this obstacle is currently "in"
    std::set<Vec2I>;
    // obstacle data...
};

struct MapChunkObject {
    /*
    local position within the chunk:
    
    with (x, y)-> * denoting a point at
    (x_rel, y_rel) within the chunk

    *-----------------------------------*
    | * <-(0.0f, 1.0f) (1.0f, 1.0f)-> * |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    |                                   |
    | * <-(0.0f, 0.0f) (1.0f, 0.0f)-> * |
    *-----------------------------------*
    */
    float x_rel, y_rel;
    std::shared_ptr<MapObstacle> obstacle;
};

/*
Represents a chunk on the map
*/
struct MapChunk {
    // the list of all the objects in this chunk
    std::vector<std::shared_ptr<MapChunkObject>> objects;
};

/*
MapChunkArray:
stores a growable 2D array of MapChunks.

NOTE: this is mostly internal to EnvMap.
*/
class MapChunkArray {
public:
    MapChunkArray();
    ~MapChunkArray();

    size_t getWidth();
    size_t getHeight();
    
    MapChunk & get_chunk(int x_offset, int y_offset, bool & in_bounds);
    const MapChunk & get_chunk(int x_offset, int y_offset, bool & in_bounds) const;

    MapChunk & create_or_get(int x_offset, int y_offset);

private:
    // the dummy chunk is a default reference to return if we can't give a valid chunk. (check in_bounds!!!!)
    static MapChunk dummy_chunk;
    // dimensions of the 2D array
    size_t width;
    size_t height;
    // actual array
    std::shared_ptr<MapChunk> * array_internal;
    
};

class EnvMap {
public:
    // which quadrant array
    enum class Quadrant : int {
        NorthEast = 0,
        NorthWest = 1,
        SouthWest = 2,
        SouthEast = 3
    };

    EnvMap();
    ~EnvMap();
    // these methods perform a spacial query of the map (global map coords in meters).
    std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRadius(float radius, float x, float y) const;
    std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinSquare(float half_width, float x, float y) const;
    std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
    // represents an index into the map arrays.
    struct XYQ {
        Vec2I xy;
        Quadrant quad;
    };
    // transforms a point in global coords (meters)
    static XYQ xyToQuadrantOffset(float x, float y);
    // transforms a quadrant into an index
    static int quadToMapArray(Quadrant quad);
    // do we have *any* map data for a given world coordinate's containing chunk?
    bool hasContainingChunk(float x, float y);
    // gets a reference to the containing chunk for a given world coordinate point
    MapChunk & getContainingChunk(float x, float y, bool & in_bounds);

    // gets the size of a quadrant
    Vec2I getQuadrantSize(Quadrant q);

    void getRobotPosition(float & x, float & y);
    void setRobotPosition(float x, float y);

private:
    MapChunkArray quadrants[4];
    float robot_x;
    float robot_y;
};

struct 
