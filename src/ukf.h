#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  // Augmented state-vector
  VectorXd x_aug_;

  ///* state covariance matrix
  MatrixXd P_; 

  // Prediction covariance matrix
  MatrixXd Ppred;

  // Augmented covariance matrix
  MatrixXd P_aug_;

  // Radar noise covariance matrix
  MatrixXd R_radar_;

  // Laser noise covariance matrix
  MatrixXd R_laser_;

  // Cross-correlation matrix
  MatrixXd Tc_radar_;
  MatrixXd Tc_laser_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  // Augmented sigma points
  MatrixXd Xsig_aug_;

  // Sigma points in measurement space
  MatrixXd Zsig_radar_;
  MatrixXd Zsig_laser_;

  // Innovation covariance matrix
  MatrixXd S_radar_;
  MatrixXd S_laser_;

  // Radar and laser Zsig and zpred difference
  VectorXd z_pred_radar_;
  VectorXd z_pred_laser_;


  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  // Measurement output states
  int n_z_radar_;
  int n_z_laser_;

  ///* Sigma point spreading parameter
  double lambda_;

  // Weights of sigma points
  VectorXd weights_;  // Weight use for predicting mean of state / covariance

  // store- previous time stamp and dt
  double prev_tstamp;
  double dt;

  // nis value for radar and laser
  double nis_radar_;
  double nis_laser_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
  // Create Augmented Sigma-Points
  */
  void AugmentedSigmaPoints();
  
  /**
  //  Make sigma-point prediction
  */
  void SigmaPointPrediction();

  /**
  //  Mean and Covariance of predicion stage.
  */
  void PredictMeanAndCovariance();
  
  /*
  //  Predict radar measurement
  */
  
  void PredictRadarMeasurement();

  /*
  //  Predict laser measurement
  */
 
  void PredictLaserMeasurement();



  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage& meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t, const MeasurementPackage& meas_package);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const MeasurementPackage& meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const MeasurementPackage& meas_package);
};

#endif /* UKF_H */
