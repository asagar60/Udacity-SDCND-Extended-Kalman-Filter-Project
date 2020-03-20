#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

   VectorXd rmse(4);

   if (estimations.size() != ground_truth.size() || estimations.size() == 0 || ground_truth.size() == 0){
     std::cout<<"Invalid estimation or ground truth data "<<std::endl;
     return rmse;
   }

   for (unsigned int i = 0; i < estimations.size(); ++i){
     VectorXd residual = estimations[i] - ground_truth[i];

     residual = residual.array() * residual.array();
     rmse+= residual;
   }

   rmse = rmse / estimations.size();
   rmse = rmse.array().sqrt();
   return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
   MatrixXd Hj = MatrixXd(3,4);

    double px = x_state[0];
    double py = x_state[1];
    double vx = x_state[2];
    double vy = x_state[3];

    double c1 = px * px + py * py;
    double c2 = sqrt(c1);
    double c3 = c1 * c2;

   if (fabs(c1) < 0.0001){
     std::cout<<"Calculate Jacobian() - Error - Division by Zero "<<std::endl;
     return Hj;
   }
   Hj << px/c2, py/c2, 0, 0,
           (-py/c1), px/c1, 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - vx*py)/c3, px/c2, py/c2;

  return Hj;

}
