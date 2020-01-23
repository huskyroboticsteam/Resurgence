#include "SyntheticLidar.h"

SyntheticLidar::SyntheticLidar() : lastAngle(0.0f),
                                   last(std::chrono::steady_clock::now()),
                                   lastFrameTime(std::chrono::steady_clock::now())
{
}

std::shared_ptr<Polar2D> SyntheticLidar::nextPolarCircle(double radius)
{
    bool frame = updateAngle();
    Polar2D pd{radius, actingAngle()};
    return completeNext(frame, pd);
}

std::shared_ptr<Polar2D> SyntheticLidar::nextPolarSquare(double dimension)
{
    bool frame = updateAngle();
    double aa = fmod(actingAngle(), 90);
    double hypotenuse;
    if (aa > 45)
    {
        hypotenuse = (dimension / 2) / cos((90 - aa) * PI / 180.0);
    }
    else
    {
        hypotenuse = (dimension / 2) / cos(aa * PI / 180.0);
    }
    Polar2D pd{hypotenuse, aa};
    return completeNext(frame, pd);
}

std::shared_ptr<Polar2D> SyntheticLidar::completeNext(bool frame, Polar2D pd)
{
    if (frame)
    {
        createFrame();
    }
    std::shared_ptr<Polar2D> spd = std::make_shared<Polar2D>(pd);
    currentFrame.push_back(spd);
    return spd;
}

std::vector<std::shared_ptr<Polar2D>> SyntheticLidar::getLastFrame()
{
    return lastFrame;
}

double SyntheticLidar::getLastAngle()
{
    return lastAngle;
}

auto SyntheticLidar::getLastFrameTime()
{
    return lastFrameTime;
}

bool SyntheticLidar::updateAngle()
{
    long millisSplit = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last).count();
    last = std::chrono::steady_clock::now();
    double angle = (double)millisSplit * 3.6;
    double lastOld = lastAngle;
    lastAngle = fmod(lastAngle, 360) + angle;
    return lastOld > lastAngle;
}

double SyntheticLidar::actingAngle()
{
    double acting = fmod(lastAngle + 360 + 90, 360); //Converts Lidar Coordinates to Polar standard.
    return acting;
}

void SyntheticLidar::createFrame()
{
    lastFrame = currentFrame;
    std::vector<std::shared_ptr<Polar2D>> newFrame;
    currentFrame = newFrame;
    lastFrameTime = std::chrono::steady_clock::now();
}