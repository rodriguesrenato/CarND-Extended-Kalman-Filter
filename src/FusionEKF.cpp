#include "FusionEKF.h"
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Constructor
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // Initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  // Measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;

  // Measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 
              0, 0.0009, 0,
              0, 0, 0.09;

  // Measurement matrix
  H_laser_ << 1, 0, 0, 0, 
              0, 1, 0, 0;

  // ekf_ = KalmanFilter();

  // Initialize state transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, 1, 0,
              0, 1, 0, 1,
              0, 0, 1, 0,
              0, 0, 0, 1;

  // Define acceleration noises
  ekf_.noise_ax_ = 9;
  ekf_.noise_ay_ = 9;
}

// Destructor
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  // Check if initialization was done before
  if (!is_initialized_) {
    ekf_.x_ = VectorXd(4);
    ekf_.P_ = MatrixXd(4, 4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      tools.Print("FusionEKF","Initialize with a Radar measurement");

      // Calculate auxiliar variables for pollar to cartesian conversion
      float range = measurement_pack.raw_measurements_[0];
      float cos_phi = std::cos(measurement_pack.raw_measurements_[1]);
      float sin_phi = std::sin(measurement_pack.raw_measurements_[1]);
      float range_rate = measurement_pack.raw_measurements_[2];

      // Convert radar from polar to cartesian coordinates
      ekf_.x_ << range * cos_phi, range * sin_phi, range_rate * cos_phi,
          range_rate * sin_phi;

      // Assuming that Radar position covariances of 10 and velocity covariances of 100
      ekf_.P_ << 10,  0,   0,   0,
                  0, 10,   0,   0,
                  0,  0, 100,   0,
                  0,  0,   0, 100;

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      tools.Print("FusionEKF","Initialize with a Lidar measurement");
      // set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0],
          measurement_pack.raw_measurements_[1], 0, 0;
      
      // Assuming that position covariances of 1 and velocity covariances of 1000 (high values due to uncertanty)
      ekf_.P_ << 1, 0,    0,    0,
                 0, 1,    0,    0,
                 0, 0, 1000,    0,
                 0, 0,    0, 1000;
    } 

    // Update previous timestamp reference
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // Done initializing
    is_initialized_ = true;
    return;
  }

  // Calculate elapsed time
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Predict the current state  
  ekf_.Predict(dt);

  // Process measurement update accorddingly to the Sensor type
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) { 
    // Update the ekf_ H and R matrices with radar matrices
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = MatrixXd(3,4);
    ekf_.R_ = MatrixXd(3,3);
    ekf_.H_ = Hj_; 
    ekf_.R_ = R_radar_;

    // Measurement update via extendend kalman filter
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Update the ekf_ H and R matrices with laser matrices
    ekf_.H_ = MatrixXd(2,4);
    ekf_.R_ = MatrixXd(2,2);
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;

    // Measurement update via standard kalman filter
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // x state and covariance P is printed at main.cpp line 144, to print values along with RMSE
}
