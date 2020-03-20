#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */

   // Initializing State covariance Matrix P
   // These values are considered from the previous example from the course.

   ekf_.P_ = MatrixXd(4, 4);
   ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;

   H_laser_ << 1,0,0,0,
              0,1,0,0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
//    ekf_.Q_ = MatrixXd(4,4);
//    ekf_.Q_ << 0,0,0,0,
//              0,0,0,0,
//              0,0,0,0,
//              0,0,0,0;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      double ro  = measurement_pack.raw_measurements_[0];
      double theta = measurement_pack.raw_measurements_[1];
      double ro_dot = measurement_pack.raw_measurements_[2];

      // Making small angle approximations . Details : https://en.wikipedia.org/wiki/Small-angle_approximation

      //float c_theta = cos(theta);
      //float s_theta = sin(theta);

      //if (c_theta < 0.0001){
      //  c_theta = 0.0001;
      //}
      //if (s_theta < 0.0001){
      // s_theta = 0.0001;
      //}
      /** SMALL ANGLE APPROXIMATION :
       * when applied small angle approximation to cos(theta) and sin(theta) only , the resulting output of px and py
       * still were small.
       * so , applying small angle approximation to entire px and py*/

      double px = ro * cos(theta);
      double py = ro * sin(theta);
      double vx = ro_dot * cos(theta);
      double vy = ro_dot * sin(theta);

      if (px < 0.0001){
          px = 0.0001;
      }
      if (py < 0.0001){
          py = 0.0001;
      }

      ekf_.x_<< px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0,0;

    }

      previous_timestamp_  = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  /** Student Note :*/
  /** when calculating the time difference, when i checked the time by dividing with 1000000 instead of 1000000.0 causes
   * SIGSEV to occur , since the outout will become int when dividing by int .
   * We need time to be float of double , so we divide it by float number 1000000.0*/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;

  ekf_.Q_ = MatrixXd(4,4);

  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.Q_(0,0) = (pow(dt, 4) * noise_ax)/4;
  ekf_.Q_(0,1) = 0;
  ekf_.Q_(0,2) = (pow(dt, 3) * noise_ax)/2;
  ekf_.Q_(0,3) = 0;
  ekf_.Q_(1,0) = 0;
  ekf_.Q_(1,1) = (pow(dt, 4) * noise_ay)/4;
  ekf_.Q_(1,2) = 0;
  ekf_.Q_(1,3) = (pow(dt, 3) * noise_ay)/2;
  ekf_.Q_(2,0) = (pow(dt, 3) * noise_ax)/2;
  ekf_.Q_(2,1) = 0;
  ekf_.Q_(2,2) = (pow(dt, 2) * noise_ax);
  ekf_.Q_(2,3) = 0;
  ekf_.Q_(3,0) = 0;
  ekf_.Q_(3,1) = (pow(dt, 3) * noise_ay)/2;
  ekf_.Q_(3,2) = 0;
  ekf_.Q_(3,3) = (pow(dt, 2) * noise_ay);


  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {

    // TODO: Laser updates_
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
