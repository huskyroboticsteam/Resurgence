#include "EnvMap.h"

#include <cmath>
#include <set>

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

const MapChunk & MapChunkArray::getChunk(const Vec2I & xy_offset, bool & in_bounds) const {
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
    },
    robot_x(0.0f),
    robot_y(0.0f),
    obstacles() {
}

EnvMap::~EnvMap() {
}

// transforms a point in global coords (meters)
XYQ EnvMap::xyToQuadrantOffset(float x, float y) {
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

/*
iterateChunkRect():
===================
This internal function accepts a low xyq (inclusive) and a high xyq (inclusive) defining a rectangle of chunks. This function
takes a function pointer (see: lambda expression) to evaluate for each valid chunk in the rectangle.
*/
void EnvMap::iterateChunkRect(XYQ low, XYQ high, std::function<void(XYQ, MapChunk &)> per_chunk) {
    switch(low.quad) {
        case Quadrant::NorthEast: {
            switch(high.quad) {
                case Quadrant::NorthEast: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; y <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                } break;
                default:
                    return;
            }
        } break;
        case Quadrant::NorthWest: {
            switch(high.quad) {
                case Quadrant::NorthWest: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthEast: {
                    for (int x = 0; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; y <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x <= low.xy.x; x ++) {
                        for (int y = low.xy.y; y <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                } break;
                default: return;
            }
        } break;
        case Quadrant::SouthEast: {
            switch(high.quad) {
                case Quadrant::SouthEast: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthEast: {
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                } break;
                default: return;
            }
        } break;
        case Quadrant::SouthWest: {
            switch(high.quad) {
                case Quadrant::SouthWest: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::SouthEast: {
                    for (int x = 0; x < low.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthWest: {
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthEast: {
                    for (int x = 0; x < low.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < low.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < high.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < high.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                } break;
            }
        } break;
    }
}

void EnvMap::iterateChunkRect(XYQ low, XYQ high, std::function<void(XYQ, const MapChunk &)> per_chunk) const {
    switch(low.quad) {
        case Quadrant::NorthEast: {
            switch(high.quad) {
                case Quadrant::NorthEast: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; y <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                } break;
                default:
                    return;
            }
        } break;
        case Quadrant::NorthWest: {
            switch(high.quad) {
                case Quadrant::NorthWest: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthEast: {
                    for (int x = 0; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; y <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x <= low.xy.x; x ++) {
                        for (int y = low.xy.y; y <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                } break;
                default: return;
            }
        } break;
        case Quadrant::SouthEast: {
            switch(high.quad) {
                case Quadrant::SouthEast: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthEast: {
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                } break;
                default: return;
            }
        } break;
        case Quadrant::SouthWest: {
            switch(high.quad) {
                case Quadrant::SouthWest: {
                    for (int x = low.xy.x; x <= high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::SouthEast: {
                    for (int x = 0; x < low.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < high.xy.x; x ++) {
                        for (int y = low.xy.y; x <= high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthWest: {
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                    for (int x = low.xy.x; x < high.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                } break;
                case Quadrant::NorthEast: {
                    for (int x = 0; x < low.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthWest}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < low.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthWest)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthWest}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < high.xy.x; x ++) {
                        for (int y = 0; y < low.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::SouthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::SouthEast}, chunk);
                            }
                        }
                    }
                    for (int x = 0; x < high.xy.x; x ++) {
                        for (int y = 0; y < high.xy.y; y ++) {
                            bool in_bounds = true;
                            const MapChunk & chunk = quadrants[static_cast<int>(Quadrant::NorthEast)].getChunk(Vec2I{x, y}, in_bounds);
                            if (in_bounds) {
                                per_chunk(XYQ {{x, y}, Quadrant::NorthEast}, chunk);
                            }
                        }
                    }
                } break;
            }
        } break;
    }
}

size_t EnvMap::newObstacleUID(const MapObstacle & obstacle) {
    size_t uid = obstacles.size();
    std::shared_ptr<MapObstacle> new_obst = std::shared_ptr<MapObstacle>(new MapObstacle(obstacle));
    new_obst->uid = uid;
    obstacles.push_back(new_obst);
    return uid;
}

std::shared_ptr<MapObstacle> EnvMap::getObstacle(size_t uid) {
    if (uid >= obstacles.size()) {
        return std::shared_ptr<MapObstacle>(nullptr);
    }
    return obstacles[uid];
}

std::vector<std::shared_ptr<const MapObstacle>> EnvMap::findObjectsWithinRadius(float radius, float x, float y) const {
    std::vector<std::shared_ptr<const MapObstacle>> out_set;
    std::set<uint64_t> found_uids;
    
    float x_low = x - radius;
    float x_high = x + radius;
    float y_low = y - radius;
    float y_high = y + radius;

    XYQ lower_left_index = xyToQuadrantOffset(x_low, y_low);
    XYQ upper_right_index = xyToQuadrantOffset(x_high, y_high);

    iterateChunkRect(lower_left_index, upper_right_index, [&](XYQ current_xyq, const MapChunk & chunk) {
        size_t chunk_obj_count = chunk.objects.size();
        for (size_t i = 0; i < chunk_obj_count; i ++) {
            const MapChunkObject & chunk_obj = chunk.objects[i];
            const MapObstacle & obstacle = * chunk_obj.obstacle;
            if (found_uids.find(obstacle.uid) == found_uids.end()) {
                float dx = x - obstacle.position.x;
                float dy = y - obstacle.position.y;
                float dist = sqrtf(dx * dx + dy * dy);
                if (dist <= radius) {
                    uint64_t obj_uid = obstacle.uid;
                    found_uids.insert(obj_uid);
                    out_set.push_back(chunk_obj.obstacle);
                }
            }
        }
    });

    return out_set;
}

std::vector<std::shared_ptr<const MapObstacle>> EnvMap::findObjectsWithinSquare(float half_width, float x, float y) const {
    float x_low = x - half_width;
    float x_high = x + half_width;
    float y_low = y - half_width;
    float y_high = y + half_width;

    return findObjectsWithinRect(x_low, y_low, x_high, y_high);
}

std::vector<std::shared_ptr<const MapObstacle>> EnvMap::findObjectsWithinRect(float x_low, float y_low, float x_high, float y_high) const {
    std::vector<std::shared_ptr<const MapObstacle>> out_set;
    std::set<uint64_t> found_uids;

    XYQ lower_left_index = xyToQuadrantOffset(x_low, y_low);
    XYQ upper_right_index = xyToQuadrantOffset(x_high, y_high);

    iterateChunkRect(lower_left_index, upper_right_index, [&](XYQ current_xyq, const MapChunk & chunk) {
        size_t chunk_obj_count = chunk.objects.size();
        for (size_t i = 0; i < chunk_obj_count; i ++) {
            const MapChunkObject & chunk_obj = chunk.objects[i];
            const MapObstacle & obstacle = * chunk_obj.obstacle;

            if (found_uids.find(obstacle.uid) == found_uids.end()) {
                if (obstacle.position.x >= x_low && obstacle.position.x <= x_high && obstacle.position.y >= y_low && obstacle.position.y <= y_high) {
                    uint64_t obj_uid = obstacle.uid;
                    found_uids.insert(obj_uid);
                    out_set.push_back(chunk_obj.obstacle);
                }
            }
        }
    });

    return out_set;
}

void EnvMap::updateObstacle(size_t uid, const MapObstacle & new_data) {
    if (uid >= obstacles.size()) {
        return;
    }
    std::shared_ptr<MapObstacle> obstacle = obstacles[uid];

    obstacle->position = new_data.position;
    obstacle->points = new_data.points;

    size_t point_count = new_data.points.size();

    std::unordered_set<XYQ> new_containing_chunks;
    for (size_t i = 0; i < point_count; i ++) {
        XYQ chunk_index_of_point = xyToQuadrantOffset(new_data.points[i].x, new_data.points[i].y);
        new_containing_chunks.insert(chunk_index_of_point);
    }

    std::unordered_set<XYQ>::const_iterator chunk_iter = new_containing_chunks.cbegin();
    while(chunk_iter != new_containing_chunks.cend()) {
        bool bad_xyq = false;
        XYQ chunk_xyq = * chunk_iter;
        MapChunk & chunk = getOrCreateChunk(chunk_xyq, bad_xyq);
        if (bad_xyq) {
            // This should never happen if xyToQuadrantOffset is implemented correctly
            throw "BAD PROGRAMMER! (xyToQuadrantOffset did the wrong thing, creating negative xy in an XYQ)";
        }

        size_t chunk_object_count = chunk.objects.size();

        bool found_ref = false;
        for (size_t i = 0; i < chunk_object_count; i ++) {
            if (chunk.objects[i].obstacle == obstacle) {
                chunk.objects[i].relative_pos = obstacle->position - (chunk_xyq.xy.into_Vec2() + Vec2(0.5f, 0.5f));
                found_ref = true;
                break;
            }
        }

        if (! found_ref) {
            chunk.objects.push_back(MapChunkObject {
                obstacle->position - (chunk_xyq.xy.into_Vec2() + Vec2(0.5f, 0.5f)),
                obstacle
            });
        }

        chunk_iter ++;
    }
}

MapChunk & EnvMap::getChunk(XYQ xyq, bool & in_bounds) {
    MapChunkArray & array = quadrants[static_cast<size_t>(xyq.quad)];
    return array.getChunk(xyq.xy, in_bounds);
}

const MapChunk & EnvMap::getChunk(XYQ xyq, bool & in_bounds) const {
    const MapChunkArray & array = quadrants[static_cast<size_t>(xyq.quad)];
    return array.getChunk(xyq.xy, in_bounds);
}

MapChunk & EnvMap::getOrCreateChunk(XYQ xyq, bool & bad_xyq) {
    MapChunkArray & array = quadrants[static_cast<size_t>(xyq.quad)];
    bool in_bounds = true;
    return array.createOrGet(xyq.xy.x, xyq.xy.y, bad_xyq);
}
