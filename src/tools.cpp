#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using Eigen::ArrayXd;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check that estimations vector size is not zero and
  // it is the same size as ground_truth zector
  if(estimations.size() == 0 || (estimations.size() != ground_truth.size())) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){
    VectorXd residuals(4);
    residuals = estimations[i] - ground_truth[i];
    residuals = residuals.array() * residuals.array();
    rmse += residuals;
  }

  //calculate the mean
  rmse = rmse / estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  
  MatrixXd Hj(3,4);

  // set state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  if(px == 0 && py == 0) {
    cout << "CalculateJacobian(): py or py is `0`. Error of division by `0`. Exiting" << endl;
    return Hj;
  }
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  Hj << (px / c2), (py / c2), 0, 0,
       -(py / c1), (px / c1), 0, 0,
         py*(vx*py - vy*px) / c3, px*(px*vy - py*vx) / c3, px / c2, py / c2;

  return Hj;
  
}
