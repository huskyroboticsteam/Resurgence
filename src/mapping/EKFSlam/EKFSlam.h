
#pragma once

#include "Eigen/Dense"
#include "common.h"
#include "measurement_package.h"
#include "../../math/PointXY.h"
#include "tools.h"
#include <set>
#include <vector>
#include <memory>

#include "../ObjectValidator.h"

#define INF 1000

struct ObstaclePoint {
  const float& x;
  const float& y;
};

struct MagnetometerReading {
  float heading; // in degrees
};

struct GPSReading {
  float x; // with respect to starting point
  float y;
};

class EKFSlam {
  public:
    /**
     * Constructor.
     */
    EKFSlam(float motion_noise = 0.6f);

    /**
     * Destructor.
     */
    virtual ~EKFSlam();


    void Prediction(const MagnetometerReading &mr, const GPSReading &gps);

    /*
     * Update state based on a reading from the lidar
     */
    void updateFromLidar(std::vector<std::set<std::shared_ptr<PointXY>>> lidarClusters);

    /*
    * Step the kalman filter forward
    * With the data that has been added so far from the update methods 
    * The kalman filter internally stores the time that the last step 
    * occurred
    */
    void step(); 

    //Returns a PointXY(x,y) of the robot's latest position
    PointXY getPosition();

    //Returns a float of the robot's latest heading
    float getHeading();
    
    // TODO: add a function(s) to access uncertainty values
    float getValidationValue(size_t id, PointXY obstacle);
    
    // Returns a vector of points of all obstacles
    std::vector<ObstaclePoint> getObstacles();

    //Returns a new id to be associated to a new landmark
    int getNewLandmarkID();

  private:
    // check whether the tracking toolbox was initialized or not (first
    // measurement)
    bool is_initialized_;
    std::vector<ObstaclePoint> obstacles;
    // tool object used to compute Jacobian and RMSE
    Tools tools;
    Eigen::MatrixXd robSigma;
    Eigen::MatrixXd robMapSigma;
    Eigen::MatrixXd mapSigma;
    Eigen::MatrixXd Sigma;
    Eigen::VectorXd mu;
    Eigen::MatrixXd Q_;
    vector<bool> observedLandmarks;
    ObjectValidator object_validator;
    void Initialize(unsigned int landmark_size, unsigned int rob_pose_size, float _motion_noise);
    int counter;
};
