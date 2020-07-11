#include "Eigen/Dense"

class StateVector : public Eigen::VectorXd {
   public:
        double &operator()(Eigen::DenseIndex i);
};