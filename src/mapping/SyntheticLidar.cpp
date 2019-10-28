#include "SyntheticLidar.h"

auto last = std::chrono::steady_clock::now(); //The current system time in milliseconds
double lastAngle(0.0f); //The angle the lidar was at for the last call to a next method.
std::vector<std::shared_ptr<Polar2D>> lastFrame; //The last completed frame.
std::vector<std::shared_ptr<Polar2D>> currentFrame; //The frame that is currently being built.
auto lastFrameTime = std::chrono::steady_clock::now(); //The system time at which the last frame was completed.

SyntheticLidar::SyntheticLidar(){

}

/**
 * Steps the Lidar for the time having passed since it was last called (Given the average time for a complete rotation of the lidar)
 * and returns the next polar coordinate at the given radius for a circle.
 * 
 * @param radius The radius for which the next point should be generated.
 * @return A shared pointer to the next Polar2D coordinate.
 */
std::shared_ptr<Polar2D> SyntheticLidar::nextPolarCircle(double radius) {
    bool frame = updateAngle();
    Polar2D pd{radius, actingAngle()};
    return completeNext(frame, pd);
}


/**
 * Steps the Lidar for the time having passed since it was last called (Given the average time for a complete rotation of the lidar)
 * and returns the next polar coordinate at the given dimension for a axis-aligned square.
 * 
 * @param radius The dimension for which the next point should be generated.
 * @return A shared pointer to the next Polar2D coordinate.
 */
std::shared_ptr<Polar2D> SyntheticLidar::nextPolarSquare(double dimension) {
    bool frame = updateAngle();
    double aa = fmod(actingAngle(), 90);
    double hypotenuse;
    if(aa>45){
        hypotenuse = (dimension/2)/cos((90-aa) * PI / 180.0);
    } else {
        hypotenuse = (dimension/2)/cos(aa * PI / 180.0);
    }
    Polar2D pd{hypotenuse, aa};
    return completeNext(frame, pd);
}

//Handles frame completion and addition.
std::shared_ptr<Polar2D> SyntheticLidar::completeNext(bool frame, Polar2D pd) {
    if(frame) createFrame();
    std::shared_ptr<Polar2D> spd = std::make_shared<Polar2D>(pd);
    currentFrame.push_back(spd);
    return spd;
}

//Gets the last completed frame or a nullptr if none have been completed.
std::vector<std::shared_ptr<Polar2D>> SyntheticLidar::getLastFrame() {
    return lastFrame;
}

//Gets the last angle the lidar generated a point at.
double SyntheticLidar::getLastAngle() {
    return lastAngle;
}

//Gets the system time at which the last frame was completed.
auto SyntheticLidar::getLastFrameTime() {
    return lastFrameTime;
}

/**
 * Updates the angle of the lidar based on the time that has passed since last it generated a point and the average speed.
 */
bool updateAngle() {
    long millisSplit = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last).count();
    last = std::chrono::steady_clock::now();
    double angle = (double) millisSplit * 3.6;
    double lastOld = lastAngle;
    lastAngle = fmod(lastAngle, 360) + angle;
    return lastOld > lastAngle;
}

//Returns the angle which should be used in a cosine equation to convert to square coordinates.
double actingAngle() {
    double acting = fmod(lastAngle + 360 + 90, 360); //Converts Lidar Coordinates to Polar standard.
}

//Copies the currentFrame into lastFrame and then clears the currentFrame so it can be built again.
void createFrame() {
    lastFrame = currentFrame;
    std::vector<std::shared_ptr<Polar2D>> newFrame;
    currentFrame = newFrame;
    lastFrameTime = std::chrono::steady_clock::now();
}