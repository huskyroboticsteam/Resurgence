#include "SyntheticLidar.h"

#include <iostream>

auto last = std::chrono::steady_clock::now();
double lastAngle(0.0f);
std::vector<std::shared_ptr<Polar2D>> lastFrame;
std::vector<std::shared_ptr<Polar2D>> currentFrame;
auto lastFrameTime = std::chrono::steady_clock::now();

SyntheticLidar::SyntheticLidar(){

}

std::shared_ptr<Polar2D> SyntheticLidar::nextPolarCircle(double radius) {
    bool frame = updateAngle();
    Polar2D pd{radius, actingAngle()};
    if(frame) createFrame();
    std::shared_ptr<Polar2D> spd = std::make_shared<Polar2D>(pd);
    std::cout << spd->r << ", " << spd->theta << std::endl;
    currentFrame.push_back(spd);
    return spd;
}

std::shared_ptr<Polar2D> SyntheticLidar::nextPolarSquare(double dimension) {
    bool frame = updateAngle();
    double aa = fmod(actingAngle(), 90);
    double hypotenuse;
    if(aa>45){
        hypotenuse = (dimension/2)/cos((90-aa) * PI / 180.0);
    } else {
        hypotenuse = (dimension/2)/cos(aa * PI / 180.0);
    }
    Polar2D pd{hypotenuse, actingAngle()};
    if(frame) createFrame();
    std::shared_ptr<Polar2D> spd = std::make_shared<Polar2D>(pd);
    currentFrame.push_back(spd);
    return spd;
}

std::vector<std::shared_ptr<Polar2D>> SyntheticLidar::getLastFrame() {
    return lastFrame;
}

double SyntheticLidar::getLastAngle() {
    return lastAngle;
}

auto SyntheticLidar::getLastFrameTime() {
    return lastFrameTime;
}

bool updateAngle() {
    long millisSplit = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last).count();
    last = std::chrono::steady_clock::now();
    double angle = (double) millisSplit * 3.6;
    double lastOld = lastAngle;
    lastAngle = fmod(lastAngle, 360) + angle;
    return lastOld > lastAngle;
}

double actingAngle() {
    double acting = fmod(lastAngle + 360 + 90, 360); //Converts Lidar Coordinates to Polar standard.
}

void createFrame() {
    lastFrame = currentFrame;
    std::vector<std::shared_ptr<Polar2D>> newFrame;
    currentFrame = newFrame;
    lastFrameTime = std::chrono::steady_clock::now();
}