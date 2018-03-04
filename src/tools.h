#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  // /**
  // * A helper method to calculate Jacobians.
  // */
  // MatrixXd CalculateJacobian(const VectorXd& x_state);

  // *
  // * A helper method to calculate the predicted measurement with the nonlinear mapping of x->y.
  
  // VectorXd ComputePredictedMeasurement(const VectorXd& x_state);

  // void unWrapVector(VectorXd& z);

  // /*
  // * helper method to unwrap angle to -pi,pi
  // */
  // double unWrapAngle(double angle);

};

#endif /* TOOLS_H_ */
