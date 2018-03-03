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
  P_init_ = MatrixXd(4,4); // Initial State Covariance Matrix
  F_init_ = MatrixXd(4,4); // Initial State Transition Matrix
  Q_init_ = MatrixXd(4,4); // Initial Process Covariance Matrix

  noise_ax = 9;
  noise_ay = 9;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // measurement model - laser 
  H_laser_ << 1, 0, 0, 0, // px
                0, 1, 0, 0; // py

  // state transition matrix
  F_init_ << 1, 0, 1, 0,
       0, 1, 1, 0,
       0, 0, 1, 0,       
       0, 0, 0, 1;

  // state covariance matrix
  P_init_ << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,       
       0, 0, 0, 10000;   

  // process covariance matrix
  Q_init_ << 1/4*noise_ax, 0, 1/2*noise_ax, 0,
       0, 1/4*noise_ay, 0, 1/2*noise_ay,
       1/2*noise_ax, 0, 1/2*noise_ax, 0,       
       0, 1/2*noise_ay, 0, noise_ay;

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
    VectorXd x_in;
    x_in = VectorXd(4);
    x_in << 1, 1, 1, 1;
    // first covariance matrix
    MatrixXd H_in;
    MatrixXd R_in;
    MatrixXd F_in;    
    MatrixXd P_in;
    MatrixXd Q_in;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho     = measurement_pack.raw_measurements_[0]; // radial distance to object
      double phi     = measurement_pack.raw_measurements_[1]; // bearing angle between object and vehicle current heading
      double rho_dot = measurement_pack.raw_measurements_[2]; // range rate
      x_in << rho*cos(phi), rho*sin(phi), rho_dot*cos(phi), rho_dot*sin(phi); // convert polar to cartesian x = rho*cos(phi), y = rho*sin(phi)
      H_in = Hj_;
      R_in = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state, process noise matrices and measurement matrices
      */
      x_in <<  measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0; // x,y distances to object (directly in cartesian coord.)
      H_in = H_laser_;            
      R_in = R_laser_;
    }

    // initializes covariance matrix
    P_in = P_init_;
    // initializes state transition matrix
    F_in = F_init_;
    // initiazlies measurement noise matrix
    Q_in = Q_init_; // init Q process matrix
    // Init the EKF 
    ekf_.Init(x_in, P_in, F_in, H_in, R_in, Q_in);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Check Initialization of EKF \n";
    cout << "Sensor Type = " << (int)measurement_pack.sensor_type_ << "\n";
    cout << "EKF x init = " << ekf_.x_ << "\n";
    cout << "EKF P init = " << ekf_.P_ << "\n";
    cout << "EKF F init = " << ekf_.F_ << "\n";
    cout << "EKF H init = " << ekf_.H_ << "\n";
    cout << "EKF R init = " << ekf_.R_ << "\n";
    cout << "EKF Q init = " << ekf_.Q_ << "\n";

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
