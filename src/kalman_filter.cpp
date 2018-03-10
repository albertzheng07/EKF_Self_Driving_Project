#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* Static Helper functions */
static double unWrapAngle(double angle);
static VectorXd computePredictedMeasurement(const VectorXd& x_state);

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /*
    Update the prediction 
  **/
  x_ = F_*x_; // x = Fx 
  P_ = F_*P_*F_.transpose() + Q_; // Update state covariance with state transtion matrix and process noise covariance (or in other words the probability update when predicting the robot's motion)
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    Update 
  */
  VectorXd z_pred = H_ * x_; // predicted measurement z
  VectorXd y = z - z_pred; // error of measured - predicted measurement
  Innovation(y);  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /** 
    Update the state by using Extended Kalman Filter equations
  */
    // 
  VectorXd z_pred = computePredictedMeasurement(x_); // predicted measurement z
  VectorXd y = z - z_pred; // error of measured - predicted measurement
  y(1) = unWrapAngle(y(1)); // unwrap angle error

  Innovation(y);
}

void KalmanFilter::Innovation(const VectorXd &y) {
  MatrixXd S = H_*P_*H_.transpose() + R_; // Probability with respect to the measurement's effect on the states
  MatrixXd K = P_*H_.transpose()*S.inverse(); // Kalman gain which fuses the two probabilities of the measurement and prediction

  x_ = x_ + K*y; // innovation or correction step
  MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
  P_ = (I-K*H_)*P_;
}



void KalmanFilter::UpdateStateTransitionMatrix(double dt)
{
  F_(0,2) = dt; // Update time dependent position components in state transition matrix
  F_(1,3) = dt;
}

void KalmanFilter::UpdateProcessCovarianceMatrix(double dt, double noise_ax, double noise_ay)
{ 
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
       0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
       dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,       
       0, dt_3/2*noise_ay, 0, dt_2*noise_ay;       
}


static VectorXd computePredictedMeasurement(const VectorXd& x_state)
{
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  VectorXd z;
  z = VectorXd(3);
  z << sqrt(px*px+py*py), atan2(py,px), (px*vx+py*vy)/sqrt(px*px+py*py); // rho, phi, rhodot

  return z;
}

static double unWrapAngle(double angle) 
{
  if (angle > M_PI)
  {
    angle = fmod(angle+M_PI,2.0*M_PI)-M_PI;
  }
  else if (angle < -M_PI)
  {
    angle = fmod(angle-M_PI,2.0*M_PI)+M_PI;
  }
  return angle; 
}

