#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  VectorXd y = z - H_ * x_;

   MatrixXd S = H_ * P_ * H_.transpose() + R_;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());

   x_ = x_ + K * y;
   P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */



    //   float px = x_[0];
//   float py = x_[1];
//   float vx = x_[2];
//   float vy = x_[3];

    // If we use float data type :- There is 50% chance for SIGSEV (Segmentation Fault to occur).
    // While debugging i found out that using double data type instead float reduces chances of segmentation fault.


    double ro = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);
    double theta = atan2(x_[1],x_[0]);
    double ro_dot = (x_[0]*x_[2] + x_[1]*x_[3])/ ro;


    VectorXd x_pred = VectorXd(3);
    x_pred<<ro,theta,ro_dot;

    VectorXd y = z - x_pred;


    //This code snippet is referred from google.com


    while ( y(1) > M_PI || y(1) < -M_PI ) {
        if ( y(1) > M_PI ) {
            y(1) -= M_PI;
        } else {
            y(1) += M_PI;
        }
    }


   MatrixXd S = H_ * P_ * H_.transpose() + R_;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());

   x_ = x_ + K * y;
   P_ = (I - K * H_) * P_;


}
