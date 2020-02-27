
#pragma once

#include "Eigen/Dense"
#include "common.h"
#include "measurement_package.h"
#include "../ObjectValidator.h"
#include "tools.h"
#define INF 1000

struct ObstaclePoint {
  const float& x;
  const float& y;
};

class EKFSLam {
  public:
    /**
     * Constructor.
     */
    EKFSLam(float motion_noise = 0.1);

    /**
     * Destructor.
     */
    virtual ~EKFSLam();

    /*
     * Update state based on a magnetometer reading
     */
    void updateFromMagnetometer(const MagnetometerReading &mr);

    /*
     * Update state based on a GPS reading
     */
    void updateFromGPS(const GPSReading &gps);

    /*
     * Update state based on a reading from the lidar
     */
    void updateFromLidar(size_t object_uid,
                         std::set<std::shared_ptr<Vec2>> reading);

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
    int EKFSLam::getNewLandmarkID();

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
    int counter;
};
