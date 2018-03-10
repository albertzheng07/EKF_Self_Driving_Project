#include <iostream>
#include "tools.h"
#include <cmath>
#include "math.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

//const double PI = 3.14;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
    * Calculate the RMSE here.
  // */
  VectorXd RMSE(4);
  RMSE << 0,0,0,0;

  if (estimations.size() != ground_truth.size() || estimations.size() == 0)
  {
  	cout << "Invalid size between estimations and ground truth \n";
  	return RMSE;
  }	

  //accumulate squared residuals
  for(uint32_t i=0; i < estimations.size(); ++i){  
  	VectorXd r = estimations[i] - ground_truth[i];  
  	//coefficient-wise multiplication
    r = r.array()*r.array(); // Sum of R^2  
    RMSE += r;
  }

  RMSE = RMSE/estimations.size(); // Get mean by averaging over n

  // square root each element of vector
  RMSE = RMSE.array().sqrt();

  return RMSE;
}

MatrixXd Tools::calculateJacobian(const VectorXd& x_state) {
  /**
    * Calculate a Jacobian here for the radar measurement function
  */
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double c1 = px*px+py*py;
  double c2 = sqrt(c1);
  double c3 = (c1*c2);

  MatrixXd H_j;
  H_j = MatrixXd(3,x_state.size());

  H_j << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2; // Jacobian matrix

  return H_j;
}