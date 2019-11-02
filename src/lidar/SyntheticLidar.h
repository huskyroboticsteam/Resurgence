#pragma once

#include <memory>
#include <chrono>
#include <math.h>
#include <vector>

struct Polar2D;

typedef struct Polar2D
{
    double r, theta;
} Polar2D;

class SyntheticLidar {
private:
    const float PI = 3.14159265;

public:
    SyntheticLidar();
    std::shared_ptr<Polar2D> nextPolarCircle(double radius);
    std::shared_ptr<Polar2D> nextPolarSquare(double dimension);
    std::vector<std::shared_ptr<Polar2D>> getLastFrame();
    double getLastAngle();
    auto getLastFrameTime();

};

bool updateAngle();
double actingAngle();
void createFrame();