#include "tools.h"
#include <iomanip>   // std::setprecision, std::setw
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

// Calculate and return the Root Mean Squared Error
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // Initialize root mean squared error vector
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check if estimation vector is valid
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    Print("CalculateRMSE","Invalid estimation or ground_truth data");
    return rmse;
  }

  // Accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // Square residual values and accumulate it in the rmse vector
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

// Calculate and return the Jacobian matrix
MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3,4);

  // Recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px+py*py;
  float c2 = std::sqrt(c1);
  float c3 = (c1*c2);

  // Check division by zero
  if (std::fabs(c1) < 0.00001) {
    Print("CalculateJacobian","Error - Division by Zero");
    return Hj;
  }

  // Compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
}

// Print a message on cout with the pattern "[origin] message"
void Tools::Print(std::string o, std::string m){
  std::cout << "[" << o << "] " << m << std::endl;
}

// Print state vector and covariance matrix on cout
void Tools::Print(Eigen::VectorXd rmse, Eigen::VectorXd x, Eigen::MatrixXd P){
  std::cout << std::fixed << std::setprecision(3);
  
  // Print rmse
  std::cout << "rmse:";
  for(int i=0;i<rmse.size();i++){
    std::cout << std::setw(7) << rmse(i) << "" ;
  }

  // Print state x
  std::cout << "\tx:";
  for(int i=0;i<x.size();i++){
    std::cout << std::setw(7) << x(i) << "" ;
  }

  // Print covariances P
  std::cout << "\tP:";
  for(int i=0;i<P.size();i++){
    std::cout << std::setw(6) << P(i) << " " ;
  }
  std::cout << std::endl;
}