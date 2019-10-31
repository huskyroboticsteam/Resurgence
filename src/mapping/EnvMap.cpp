#include "EnvMap.h"

#include <cmath>

MapChunkArray::MapChunkArray():
    width(0),
    height(0) ,
    array(nullptr) {
}

MapChunkArray::~MapChunkArray() {
    if (array != nullptr) {
        delete[] array;
    }
}

MapChunk MapChunkArray::dummy_chunk;

size_t MapChunkArray::getWidth() {
    return width;
}

size_t MapChunkArray::getHeight() {
    return height;
}

MapChunk & MapChunkArray::getChunk(const Vec2I & xy_offset, bool & in_bounds) {
    if (xy_offset.x < 0 || xy_offset.x >= width || xy_offset.y < 0 || xy_offset.y >= height) {
        in_bounds = false;
        return dummy_chunk;
    }
    return * array[xy_offset.x + xy_offset.y * width];
}

const MapChunk & MapChunkArray::getChunk(const Vec2I & xy_offset, int y_offset, bool & in_bounds) const {
    if (xy_offset.x < 0 || xy_offset.x >= width || xy_offset.y < 0 || xy_offset.y >= height) {
        in_bounds = false;
        return dummy_chunk;
    }
    return * array[xy_offset.x + xy_offset.y * width];
}

MapChunk & MapChunkArray::getDummyChunk() {
    return dummy_chunk;
}

MapChunk & MapChunkArray::createOrGet(int x_offset, int y_offset, bool & in_bounds) {
    if (x_offset < 0 || y_offset < 0) {
        in_bounds = false;
        return dummy_chunk;
    }
    resize(x_offset, y_offset);
    return *array[x_offset + y_offset * width];
}

void MapChunkArray::resize(int max_x_offset, int max_y_offset) {
    size_t new_width = (size_t) max_x_offset + 1;
    size_t new_height = (size_t) max_y_offset + 1;
    // this allows for amortized O(log(n)) allocation/resize time
    new_width *= 2;
    new_height *= 2;

    if (width > new_width) {
        new_width = width;
    }
    if (height > new_height) {
        new_height = height;
    }

    std::shared_ptr<MapChunk> * new_array = new std::shared_ptr<MapChunk>[new_width * new_height];
    if (array != nullptr) {
        for (int x = 0; x < width; x ++) {
            for (int y = 0; y < height; y ++) {
                new_array[x + y * new_width] = array[x + y * width];
            }
        }
        for (int x = width; x < new_width; x ++) {
            for (int y = 0; y < height; y ++) {
                new_array[x + y * new_width].reset(new MapChunk());
            }
        }
        for (int x = width; x < new_width; x ++) {
            for (int y = height; y < new_height; y ++) {
                new_array[x + y * new_width].reset(new MapChunk());
            }
        }
        for (int x = 0; x < width; x ++) {
            for (int y = height; y < new_height; y ++) {
                new_array[x + y * new_width].reset(new MapChunk());
            }
        }
        delete[] array;
    }
    array = new_array;
    width = new_width;
    height = new_height;
}

EnvMap::EnvMap():
    quadrants {
        MapChunkArray(),
        MapChunkArray(),
        MapChunkArray(),
        MapChunkArray()
    } {
}

EnvMap::~EnvMap() {
}

// transforms a point in global coords (meters)
EnvMap::XYQ EnvMap::xyToQuadrantOffset(float x, float y) {
    float x_abs = fabs(x);
    float y_abs = fabs(y);
    bool x_sign = x >= 0.0f;
    bool y_sign = y >= 0.0f;
    Quadrant quad = Quadrant::NorthEast;
    if (x_sign) {
        if (y_sign) {
            quad = Quadrant::NorthEast;
        } else {
            quad = Quadrant::SouthEast;
        }
    } else {
        if (y_sign) {
            quad = Quadrant::NorthWest;
        } else {
            quad = Quadrant::SouthWest;
        }
    }
    Vec2I xy_off = Vec2I{(int) x_abs, (int) y_abs};
    return XYQ {
        xy_off,
        quad
    };
}

// transforms a quadrant into an index
int EnvMap::quadToMapArray(Quadrant quad) {
    return static_cast<int>(quad);
}
// do we have *any* map data for a given world coordinate's containing chunk?
bool EnvMap::hasContainingChunk(float x, float y) {
    XYQ index = xyToQuadrantOffset(x, y);
    MapChunkArray & array = quadrants[quadToMapArray(index.quad)];
    if (array.getWidth() <= index.xy.x || array.getHeight() <= index.xy.y) {
        return false;
    }
    return true;
}
// gets a reference to the containing chunk for a given world coordinate point
MapChunk & EnvMap::getContainingChunk(float x, float y, bool & in_bounds) {
    XYQ index = xyToQuadrantOffset(x, y);
    MapChunkArray & array = quadrants[quadToMapArray(index.quad)];
    return array.getChunk(index.xy, in_bounds);
}

// gets the size of a quadrant
Vec2I EnvMap::getQuadrantSize(Quadrant q) {
    MapChunkArray & array = quadrants[quadToMapArray(q)];
    return Vec2I{(int) array.getWidth(), (int) array.getHeight()};
}

void EnvMap::getRobotPosition(float & x, float & y) {
    x = robot_x;
    y = robot_y;
}
void EnvMap::setRobotPosition(float x, float y) {
    robot_x = x;
    robot_y = y;
}
