#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
 public:
  // Constructor.
  Tools();

  // Destructor
  ~Tools();
  
  // Helper method to calculate RMSE.
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  // Helper method to calculate Jacobians.
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

  // Helper method to print a string message on cout
  void Print(std::string o, std::string m);

  // Helper method to print the given state vector and convariance matrix on cout
  void Print(Eigen::VectorXd rmse, Eigen::VectorXd x, Eigen::MatrixXd P);
};

#endif  // TOOLS_H_
