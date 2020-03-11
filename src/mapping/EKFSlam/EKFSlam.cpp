#include "EKFSlam.h"
#include "../ObjectValidator.h"

/*
 * Constructor.
 */
EKFSlam::EKFSlam(float motion_noise) : object_validator(*this) {
  is_initialized_ = false;
  counter = 0;
  this->Initialize(10, 3, motion_noise);
}

/**
Initialize the parameters
**/

void EKFSlam::Initialize(unsigned int landmark_size, unsigned int rob_pose_size, float _motion_noise) {
  
  int N       = landmark_size; //number of landmarks
  int r       = rob_pose_size; //number of robot states(3)  
  mu          = VectorXd::Zero(2*N + r, 1); //state vector, x,y,angle,obs
  robSigma    = MatrixXd::Zero(r, r);
  robMapSigma = MatrixXd::Zero(r, 2*N);
  mapSigma    = INF*MatrixXd::Identity(2*N, 2*N);
  Sigma = MatrixXd::Zero(2*N + r, 2*N + r); //Covariance Matrix

  Sigma.topLeftCorner(r, r)          = robSigma;
  Sigma.topRightCorner(r, 2*N)       = robMapSigma;
  Sigma.bottomLeftCorner(2*N, r)     = robMapSigma.transpose();
  Sigma.bottomRightCorner(2*N, 2*N)  = mapSigma;

  float motion_noise =_motion_noise;

  Q_  = MatrixXd::Zero(2*N + r, 2*N + r);
  Q_.topLeftCorner(3,3) << motion_noise, 0, 0,
        0, motion_noise , 0,
        0, 0,   motion_noise/10;

  observedLandmarks.resize(N);
  fill(observedLandmarks.begin(), observedLandmarks.end(), false);

}


/**
* Destructor.
*/
EKFSlam::~EKFSlam() {}


void EKFSlam::Prediction(const MagnetometerReading &mr, const GPSReading &gps) {
//take in motion of robot
//use update from magnetometer and gps here
  //double angle = mu(2);
  //double r1    = motion.r1;
  //double t     = motion.t;
  //double r2    = motion.r2;

  MatrixXd Gt = MatrixXd(3,3); //Matrix A(jacobian of prediction model)
  Gt << 1, 0, -(gps.y - mu(1)),
        0, 1,  gps.x - mu(0),
        0, 0,  0;

  //float c = cos(angle + r1);
  //float s = sin(angle + r1);

  mu(0) = gps.x; //update x
  mu(1) = gps.y;  //update y
  mu(2) = mr.heading;  //update heading
  
  //Update Covariance Matrix
  int size = Sigma.cols();
  Sigma.topLeftCorner(3,3) = Gt * Sigma.topLeftCorner(3,3) * Gt.transpose();
  Sigma.topRightCorner(3, size-3) = Gt * Sigma.topRightCorner(3, size-3);
  Sigma.bottomLeftCorner(size-3, 3) = Sigma.topRightCorner(3, size-3).transpose();
  Sigma = Sigma + Q_; 

}


void EKFSlam::updateFromLidar(std::vector<std::vector<PointXY>>& lidarClusters) {
//takes in observations, which is a vector of readings, has id, range, bearing
//call validator here, validator will associate lidar readings with existing obstacles
//returns a vector of ids corresponding to the lidar readings
//validator will assign a new id to a new obstacle
  std::vector<PointXY> boxes = object_validator.boundingBox(lidarClusters, 1);
  std::vector<size_t> ids = object_validator.validate(boxes);
  // number of measurements in this step
  int m = ids.size();
  assert(m == boxes.size());
  //[range, bearing, range, bearing, .....]
  VectorXd Z          = VectorXd::Zero(2*m);
  VectorXd expectedZ  = VectorXd::Zero(2*m);

  //Jacobian matrix;
  int N = observedLandmarks.size();
  MatrixXd H = MatrixXd::Zero(2*m, 2*N + 3);
  for (int i = 0; i < m; i++) { //cycle through observations

      PointXY& reading = boxes[i];
      size_t landmarkId = ids[i]; //change to take from validator
      //landmark is not seen before(has a new id), so to initialize the landmarks
      if (!observedLandmarks[landmarkId-1]) {
          mu(2*landmarkId + 1) = reading.x;
          mu(2*landmarkId+2) = reading.y;
          //Indicate in the observedLandmarks vector that this landmark has been observed
          observedLandmarks[landmarkId-1] = true;
      }

      //add the landmark meansurement to the Z vector
      Z(2*i) = std::sqrt(std::pow(mu(0) - reading.x, 2) + std::pow(mu(1) - reading.y, 2));
      Z(2*i+1) = atan2f(mu(1) - reading.y, mu(0) - reading.x);
      //use the current estimate of the landmark poseq
      double deltax = mu(2*landmarkId+1) - mu(0);
      double deltay = mu(2*landmarkId+2) - mu(1);
      double q      = pow(deltax, 2) + pow(deltay, 2);
      expectedZ(2*i) = sqrt(q);
      expectedZ(2*i+1)   = atan2(deltay, deltax) - mu(2);

      H.block<2,3>(2*i,0) << -sqrt(q)*deltax/q, -sqrt(q)*deltay/q, 0,
                              deltay/q, -deltax/q, -1;
      H.block<2,2>(2*i, 2*landmarkId + 1) << sqrt(q)*deltax/q, sqrt(q)*deltay/q,
                                                -deltay/q, deltax/q; 
  }
  // cout << mu.transpose() << endl;
  //cout << H << endl;
  //construct the sensor noise 
  MatrixXd Q = MatrixXd::Identity(2*m, 2*m)*0.01;
  //compute the Kalman gain
  MatrixXd Ht = H.transpose();
  HQ = (H*Sigma*Ht) + Q; //Innovation Covariance (S)
  MatrixXd Si = HQ.inverse();
  //Kalman Gain
  //two columns, range and bearing for robot state and each landmark
  MatrixXd K = Sigma*Ht*Si;  
  //update 

  diff = Z - expectedZ; //(z-h) = v innovation?
  tools.normalize_bearing(diff);
  mu = mu + K * diff; //update state vector
  Sigma = Sigma - K*H*Sigma; //update covariance matrix
  mu(2) = tools.normalize_angle(mu(2));
}

/*void EKFSlam::ProcessMeasurement(const Record& record) {

      Prediction(record.odo); 
      Correction(record.radars);
}*/

std::vector<ObstaclePoint> EKFSLam::getObstacles() {
  for(int i = obstacles.size + 3; i < mu.size; i = i + 2) {
    ObstaclePoint ob{mu[i], mu[i+1]};
    obstacles.push_back(ob);
  }
  return obstacles;
}

PointXY EKFSlam::getPosition() {
  PointXY location{mu[0], mu[1]};
  return location;
}

float EKFSlam::getHeading() {
  return mu[2];
}

float EKFSlam::getValidationValue(size_t id, PointXY obstacle) {
//innovation covariance:HQ
//v^T * HQ^-1 * v < lambda
//v = innovation
  return 0;

}

int EKFSlam::getNewLandmarkID() {
  counter++;
  return counter;
}