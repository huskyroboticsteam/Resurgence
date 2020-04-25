#include "StateVector.h"

double &StateVector::operator()(Eigen::DenseIndex i) {
   return Eigen::VectorXd::operator()(i); 
}