#pragma once

#include <memory>
#include <vector>
#include <unordered_set>
#include <functional>

constexpr float ChunkDimensionMeters = 1.0f;

// basic representation of an integer {x, y} pair
struct Vec2I {
    int x;
    int y;

    inline bool operator==(const Vec2I & other) {
        return (x == other.x && y == other.y);
    }
};

namespace std {
    template<> struct hash<Vec2I> {
        std::size_t operator()(const Vec2I & v) const noexcept {
            return (size_t) (0x10000 * v.y + v.x);
        }
    };
}

class IObstacle {
public:
    inline virtual ~IObstacle() = default;

    virtual float getXY(float & x, float & y) const = 0;
    virtual void getSurroundingChunks(std::vector<Vec2I> & chunk_offsets_out) const = 0;
};

struct MapObstacle {
    // MapObstacle must be default-constructible and copy-constructible
    MapObstacle() = default;
    MapObstacle(const MapObstacle & copy_from) = default;
    // global position relative to start position
    float x = 0.0f, y = 0.0f;
    // the set of chunks this obstacle is currently "in"
    std::unordered_set<Vec2I> containing_chunks;
    // the unique identifier for this obstacle
    size_t uid;
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
    
    MapChunk & getChunk(const Vec2I & xy_offset, bool & in_bounds);
    const MapChunk & getChunk(const Vec2I & xy_offset, bool & in_bounds) const;

    static MapChunk & getDummyChunk();

    MapChunk & createOrGet(int x_offset, int y_offset, bool & in_bounds);

private:
    // the dummy chunk is a default reference to return if we can't give a valid chunk. (check in_bounds!!!!)
    static MapChunk dummy_chunk;
    // dimensions of the 2D array
    size_t width;
    size_t height;
    // actual array
    std::shared_ptr<MapChunk> * array;
    // grow the array
    void resize(int new_width, int new_height);
    
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
    std::vector<std::shared_ptr<const MapObstacle>> findObjectsWithinRadius(float radius, float x, float y) const;
    std::vector<std::shared_ptr<const MapObstacle>> findObjectsWithinSquare(float half_width, float x, float y) const;
    std::vector<std::shared_ptr<const MapObstacle>> findObjectsWithinRect(float x_lower_left, float y_lower_left, float x_upper_right, float y_upper_right) const;
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
    // gets/sets robot position
    void getRobotPosition(float & x, float & y);
    void setRobotPosition(float x, float y);
    // create a new obstacle and get it's uid
    size_t newObstacleUID(const MapObstacle & proto);
    // gets an obstacle (or none) based on uid
    std::shared_ptr<MapObstacle> getObstacle(size_t uid);

private:
    MapChunkArray quadrants[4];
    float robot_x;
    float robot_y;
    std::vector<std::shared_ptr<MapObstacle>> obstacles;

    void iterateChunkRect(XYQ low, XYQ high, std::function<void(XYQ, const MapChunk &)> per_chunk) const;
    void iterateChunkRect(XYQ low, XYQ high, std::function<void(XYQ, MapChunk &)> per_chunk);
};
