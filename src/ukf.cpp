#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <iomanip>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  // Set is_initialized_ flag
   is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  n_x_ = 5;
  n_aug_ = 7;

  // Number of measurement output states
  n_z_radar_ = 3;
  n_z_laser_ = 2;

  x_ = VectorXd(n_x_);
  x_aug_ = VectorXd(n_aug_);

  // initial covariance matrix for measurement update
  // and prediction state
  P_ = MatrixXd(n_x_, n_x_);
  Ppred = MatrixXd(n_x_, n_x_);
  P_.fill(0);
  Ppred.fill(0);


  // Initial augmented covariance matrix
  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0);


  // Sigma-points
  Xsig_pred_ = MatrixXd(this->n_x_, 2 * this->n_aug_ + 1);
  Xsig_pred_.fill(0);


  // Augmented sigma point
  Xsig_aug_ = MatrixXd(this->n_aug_, 2 * this->n_aug_ + 1);
  Xsig_aug_.fill(0);



  // Generate measurement sigma-points
  Zsig_radar_ = MatrixXd(n_z_radar_, 2 * n_aug_ + 1);
  Zsig_laser_ = MatrixXd(n_z_laser_, 2 * n_aug_ + 1);
  Zsig_radar_.fill(0);
  Zsig_laser_.fill(0);

  // Innovation covariance matrix
  S_radar_ = MatrixXd(n_z_radar_, n_z_radar_);
  S_laser_ = MatrixXd(n_z_laser_, n_z_laser_);
  S_radar_.fill(0);
  S_laser_.fill(0);

  // Radar and laser Zsig and zpred difference
  z_pred_radar_ = VectorXd(n_z_radar_);
  z_pred_laser_ = VectorXd(n_z_laser_);
  z_pred_radar_.fill(0);
  z_pred_laser_.fill(0);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  

  // Initialize measurement noise matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_,   0,                  0,
	             0,          std_radphi_*std_radphi_,    0,
	             0,                0,                 std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_,                     0,
	                 0,                       std_laspy_ * std_laspy_;

  Tc_radar_ = MatrixXd(n_x_, 3);
  Tc_laser_ = MatrixXd(n_x_, 2);

  // Initialize nis value
  nis_radar_ = 0.0;
  nis_laser_ = 0.0;

  // Initialize lambda_ value
  lambda_ = 3 - n_aug_;

  //set vector for weights and pre-compute weight
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  for (int i = 1; i<2 * n_aug_ + 1; i++) {  //2n+1 weights
	  weights_(i) = 0.5 / (n_aug_ + lambda_);
  }



}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage& meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
	if (!this->is_initialized_)
	{
		// Initialized state
		this->x_.fill(0);
		this->x_aug_.fill(0);

		this->P_ << 1, 0, 0, 0, 0,
			0, 1, 0, 0, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1;

		// Initialized first measurement
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		{
			// Initialize with radar-data
			double rho = meas_package.raw_measurements_[0];
			double theta = meas_package.raw_measurements_[1];    // bore angle
			double rho_dot = meas_package.raw_measurements_[2];  

			this->x_(0) = rho * cos(theta);   // px
			this->x_(1) = rho * sin(theta);   // py
			this->x_(2) = rho_dot;            // Assume velocity equal to rho_dot
			this->x_(3) = 0;                  // yaw-angle       (rad)
			this->x_(4) = 0;                  // yaw-angle rate  (rad/sec)

		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		{
			// Initialize with Lidar data
			this->x_(0) = meas_package.raw_measurements_[0];
			this->x_(1) = meas_package.raw_measurements_[1];
			this->x_(2) = 0;
			this->x_(3) = 0;
			this->x_(4) = 0;
		}
		this->is_initialized_ = true;
		this->prev_tstamp = meas_package.timestamp_;
		this->dt = 0.0;
		return;
	}

	//compute the time elapsed between the current and previous measurements
	this->dt = (meas_package.timestamp_ - this->prev_tstamp) / 1000000.0;	//dt - expressed in seconds
	this->prev_tstamp = meas_package.timestamp_;

	if (double(dt) > 2.0)
	{
		is_initialized_ = false;
	}

	// Limit dt to be within 0.1 sec max...
	dt = dt > 0.0 ? dt : 0.0;
	dt = dt < 0.1 ? dt : 0.1;


	// Call Prediction function
	// ===========================
	Prediction(dt, meas_package);

	// Call Predict and Update Measurement function
	// ==============================================
	if (meas_package.sensor_type_ == MeasurementPackage::LASER)
	{
		PredictLaserMeasurement();
		UpdateLidar(meas_package);
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
	{
		PredictRadarMeasurement();
		UpdateRadar(meas_package);
	}

}




/*******************************************************************************
* Programming assignment functions:
*******************************************************************************/

void UKF::AugmentedSigmaPoints() {

	//define spreading parameter
	double lambda = 3 - n_aug_;

	//create augmented mean state
	x_aug_.head(5) = x_;
	x_aug_(5) = 0;
	x_aug_(6) = 0;

	//create augmented covariance matrix
	P_aug_.fill(0.0);
	P_aug_.topLeftCorner(5, 5) = P_;
	P_aug_(5, 5) = std_a_ * std_a_;
	P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug_.llt().matrixL();

	//create augmented sigma points
	Xsig_aug_.col(0) = x_aug_;
	for (int i = 0; i< n_aug_; i++)
	{
		Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda + n_aug_) * L.col(i);
		Xsig_aug_.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda + n_aug_) * L.col(i);
	}

}

/*  =========================  
//  Sigma Point prediction
   ==========================
*/

void UKF::SigmaPointPrediction() {



	for (int i = 0; i< 2 * n_aug_ + 1; i++)
	{
		//extract values for better readability
		double p_x = Xsig_aug_(0, i);
		double p_y = Xsig_aug_(1, i);
		double v = Xsig_aug_(2, i);
		double yaw = Xsig_aug_(3, i);
		double yawd = Xsig_aug_(4, i);
		double nu_a = Xsig_aug_(5, i);
		double nu_yawdd = Xsig_aug_(6, i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd * this->dt) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * this->dt));
		}
		else {
			px_p = p_x + v * this->dt*cos(yaw);
			py_p = p_y + v * this->dt*sin(yaw);
		}

		double v_p = v;
		double yaw_p = yaw + yawd * this->dt;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5*nu_a*this->dt*this->dt * cos(yaw);
		py_p = py_p + 0.5*nu_a*this->dt*this->dt * sin(yaw);
		v_p = v_p + nu_a * this->dt;

		yaw_p = yaw_p + 0.5*nu_yawdd*this->dt*this->dt;
		yawd_p = yawd_p + nu_yawdd * this->dt;

		//write predicted sigma point into right column
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;

	}

}


// =============================================
//  Function : PredictMaenAndCovariance        =
// =============================================

void UKF::PredictMeanAndCovariance() {

	/*******************************************************************************
	* Student part begin
	******************************************************************************/
	VectorXd x_diff;

	//predicted state mean
	x_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}

	//predicted state covariance matrix
	Ppred.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

											   // state difference
		 x_diff = Xsig_pred_.col(i) - x_;


		//angle normalization
		// Instead of using two while-loop, use sin function 
		// to speed up computation. Using small angle
		// analysis, sin(dtheta) for small dtheta, will produce
		// result that is close to dtheta.  Since it is expected
		// x_diff(3) shrinks to zero over-time, sin(x_diff(3)) in
		// general will be close to normalized x_diff(3) value 

		x_diff(3) = sin(x_diff(3));
	//	while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
	//	while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		Ppred = Ppred + weights_(i) * x_diff * x_diff.transpose();
	}

}

// Predicting radar measurement
void UKF::PredictRadarMeasurement() {

	/*******************************************************************************
	* Student part begin
	******************************************************************************/

	//transform sigma points into measurement space
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

											   // extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Zsig_radar_(0, i) = sqrt(p_x*p_x + p_y * p_y);                        //r
		Zsig_radar_(1, i) = atan2(p_y, p_x);                                 //phi
		Zsig_radar_(2, i) = (p_x*v1 + p_y * v2) / sqrt(p_x*p_x + p_y * p_y);   //r_dot
	}

	//mean predicted measurement
	z_pred_radar_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred_radar_ = z_pred_radar_ + weights_(i) * Zsig_radar_.col(i);
	}

	//innovation covariance matrix S
	S_radar_.fill(0.0);
	VectorXd z_diff_radar_(n_z_radar_);

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
											   //residual
		z_diff_radar_ = Zsig_radar_.col(i) - z_pred_radar_;

		//angle normalization
		// Instead of using two while-loop, use sin function 
		// to speed up computation. Using small angle
		// analysis, sin(dtheta) for small dtheta, will produce
		// result that is close to dtheta.  Since it is expected
		// z_diff(1) shrinks to zero over-time, sin(z_diff(1)) in
		// general will be close to normalized z_diff(1) value 
		z_diff_radar_(1) = sin(z_diff_radar_(1));

//		while (z_diff_radar_(1)> M_PI) z_diff_radar_(1) -= 2.*M_PI;
//		while (z_diff_radar_(1)<-M_PI) z_diff_radar_(1) += 2.*M_PI;

		S_radar_ = S_radar_ + weights_(i) * z_diff_radar_ * z_diff_radar_.transpose();
	}

	//add measurement noise covariance matrix
	S_radar_ = S_radar_ + R_radar_;

}


// Predicting radar measurement
void UKF::PredictLaserMeasurement() {

	//transform sigma points into measurement space
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

												// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1 = cos(yaw)*v;
		double v2 = sin(yaw)*v;

		// measurement model
		Zsig_laser_(0, i) = p_x;      //px
		Zsig_laser_(1, i) = p_y;      // py
	}

	//mean predicted measurement
	z_pred_laser_.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred_laser_ = z_pred_laser_ + weights_(i) * Zsig_laser_.col(i);
	}

	//innovation covariance matrix S
	S_laser_.fill(0.0);
	VectorXd z_diff_laser(n_z_laser_);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
												//residual
		z_diff_laser = Zsig_laser_.col(i) - z_pred_laser_;
		S_laser_ = S_laser_ + weights_(i) * z_diff_laser * z_diff_laser.transpose();
	}

	//add measurement noise covariance matrix
	S_laser_ = S_laser_ + R_laser_;

}


// ====================
// Update-state       =
// ====================

void UKF::UpdateRadar(const MeasurementPackage& meas_package) {

	VectorXd z(n_z_radar_);

	/*******************************************************************************
	* Student part begin
	******************************************************************************/

	// Populate radar measurement data
	z(0) = meas_package.raw_measurements_(0);   // rho (meters)
	z(1) = meas_package.raw_measurements_(1);   // bore-angle (radians)
	z(2) = meas_package.raw_measurements_(2);   // rhodot, range-rate (m/sec)

	//calculate cross correlation matrix
	Tc_radar_.fill(0.0);
	VectorXd z_diff;
	VectorXd x_diff;

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

											   //residual
		z_diff = Zsig_radar_.col(i) - z_pred_radar_;
		//angle normalization
		while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

		// state difference
		x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		x_diff(3) = sin(x_diff(3));
//		while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
//		while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		Tc_radar_ = Tc_radar_ + ( weights_(i) * x_diff * z_diff.transpose() );
	}

	//Kalman gain K;
	MatrixXd K = Tc_radar_ * S_radar_.inverse();

	//residual
	z_diff = z - z_pred_radar_;

	//angle normalization
	z_diff(1) = sin(z_diff(1));
//	while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
//	while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = Ppred - K * S_radar_*K.transpose();

	// Compute nis
	nis_radar_ = (z - z_pred_radar_).transpose() * S_radar_.inverse() * (z - z_pred_radar_);

}


void UKF::UpdateLidar(const MeasurementPackage& meas_package) {

	VectorXd z(n_z_laser_);

	/*******************************************************************************
	* Student part begin
	******************************************************************************/

	// Populate radar measurement data
	z(0) = meas_package.raw_measurements_(0);   // px (meters)
	z(1) = meas_package.raw_measurements_(1);   // py (meters)


    //calculate cross correlation matrix
	Tc_laser_.fill(0.0);
	VectorXd z_diff;
	VectorXd x_diff;

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

												//residual
		z_diff = Zsig_laser_.col(i) - z_pred_laser_;

		// state difference
		x_diff = Xsig_pred_.col(i) - x_;

		// angle normalization
		// Instead of using two while-loop, use sin function 
		// to speed up computation. Using small angle
		// analysis, sin(dtheta) for small dtheta, will produce
		// result that is close to dtheta.  Since it is expected
		// x_diff(3) shrinks to zero over-time, sin(x_diff(3)) in
		// general will be close to normalized x_diff(3) value 
		x_diff(3) = sin(x_diff(3));

	//	while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
	//	while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

		Tc_laser_ = Tc_laser_ + weights_(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc_laser_ * S_laser_.inverse();

	//residual
	z_diff = z - z_pred_laser_;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = Ppred - K * S_laser_*K.transpose();

	// Compute nis
	nis_laser_ = (z - z_pred_laser_).transpose() * S_laser_.inverse() * (z - z_pred_laser_);

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t, const MeasurementPackage& meas_package) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	AugmentedSigmaPoints();
	SigmaPointPrediction();
	PredictMeanAndCovariance();
}
