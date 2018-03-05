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
  	RMSE += r.cwiseProduct(r); // Sum of R^2	
  }

  RMSE = RMSE/estimations.size(); // Get mean by averaging over n

  // square root each element of vector
  RMSE = RMSE.array().sqrt();

  return RMSE;
}