#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);   // jacobian measurement model - radar
  Q_v_ = MatrixXd(2,2); // covariance matrix of individual noise processes ax, ay
  G_ = MatrixXd(4,2);   // kinematic model of process noise covariance to map back to state vector

  F_init_ = MatrixXd(4,4); // Initial State Transition Matrix

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // measurement model - laser 
  H_laser_ = << 1, 0, 0, 0, // px
                0, 1, 0, 0; // py

  // process noise covariance matrix
  Q_v_ << 9, 0;
              0, 9;

  // state transition matrix
  F_init_ << 1, 0, previous_timestamp_, 0,
       0, 1, previous_timestamp_, 0,
       0, 0, 1, 0,       
       0, 0, 0, 1;       
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    x_in = VectorXd(4);
    // first covariance matrix

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    }

    // initializes covariance matrix
    P_in = MatrixXd(4,4);
    // initializes state transition matrix
    F_in = F_init_;
    // initializes measurement model matrix
    H_in = MatrixXd(5,4); // 2 x 4 for laser, 3 x 4 for radar
    // initializes process noise matrix
    R_in = MatrixXd(5,4); //
    // initiazlies measurement noise matrix
    Q_in = Q_accel_;

    // Init the EKF 
    ekf.Init(&x_in,&P_in, &F_in, &H_in, &R_in, &Q_in);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}

void EKF::updateProcessKinematicModel(const long long dt)
{
    G_ << dt*dt/2.0, 0, 
          0, dt*dt/2.0, 
          dt, 0, 
          0 , dt;         
}
