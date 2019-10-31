#pragma once

#include <memory>
#include <vector>
#include <set>

constexpr float ChunkDimensionMeters = 1.0f;

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

struct MapChunk {
    std::vector<std::shared_ptr<MapChunkObject>> objects;
};

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
    enum class Quadrant : int {
        NorthEast = 0,
        NorthWest = 1,
        SouthWest = 2,
        SouthEast = 3
    };

    EnvMap();
    ~EnvMap();

    std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRadius(float radius, float x, float y) const;
    std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinSquare(float half_width, float x, float y) const;
    std::vector<std::shared_ptr<MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
    
    struct XYQ {
        int x_offset, y_offset;
        Quadrant quad;
    };

    XYQ xyToQuadrantOffset(float x, float y);

    MapChunk & get_chunk(int x_offset, int y_offset);
    const MapChunk & get_chunk(int x_offset, int y_offset) const;
    
    bool has_contianing_chunk(float x, float y);

    Vec2I get_quadrant_size(Quadrant q);

private:
    MapChunkArray quadrants[4];
};

struct 
