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
    //Handles frame completion and addition.
    std::shared_ptr<Polar2D> completeNext(bool frame, Polar2D pd);
    /**
     * Updates the angle of the lidar based on the time that has passed since last it generated a point and the average speed.
     */
    bool updateAngle();
    //Returns the angle which should be used in a cosine equation to convert to square coordinates.
    double actingAngle();
    //Copies the currentFrame into lastFrame and then clears the currentFrame so it can be built again.
    void createFrame();
    std::chrono::time_point<std::chrono::steady_clock> last; //The current system time in milliseconds
    double lastAngle; //The angle the lidar was at for the last call to a next method.
    std::vector<std::shared_ptr<Polar2D>> lastFrame; //The last completed frame.
    std::vector<std::shared_ptr<Polar2D>> currentFrame; //The frame that is currently being built.
    std::chrono::time_point<std::chrono::steady_clock> lastFrameTime; //The system time at which the last frame was completed.

public:
    SyntheticLidar();
    /**
     * Steps the Lidar for the time having passed since it was last called (Given the average time for a complete rotation of the lidar)
     * and returns the next polar coordinate at the given radius for a circle.
     * 
     * @param radius The radius for which the next point should be generated.
     * @return A shared pointer to the next Polar2D coordinate.
     */
    std::shared_ptr<Polar2D> nextPolarCircle(double radius);
    /**
     * Steps the Lidar for the time having passed since it was last called (Given the average time for a complete rotation of the lidar)
     * and returns the next polar coordinate at the given dimension for a axis-aligned square.
     * 
     * @param radius The dimension for which the next point should be generated.
     * @return A shared pointer to the next Polar2D coordinate.
     */
    std::shared_ptr<Polar2D> nextPolarSquare(double dimension);
    //Gets the last completed frame or a nullptr if none have been completed.
    std::vector<std::shared_ptr<Polar2D>> getLastFrame();
    //Gets the last angle the lidar generated a point at.
    double getLastAngle();
    //Gets the system time at which the last frame was completed.
    //auto getLastFrameTime();
};
