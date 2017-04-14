#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::cout;
using std::endl;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // initial state vector [px, py, v, yaw, yawRate]
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.8;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = std_laspx_; // 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  
  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = n_x_ + 2;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  ///* the current NIS for radar
  NIS_radar_ = 0.0;

  ///* the current NIS for laser
  NIS_laser_ = 0.0;

  //* Set initialization to false
  is_initialized_ = false;

  //* Initilaize previous timestamp to 0
  previous_timestamp_ = 0;
}

UKF::~UKF() {}

/**
 * UKF::ProcessMeasurement(MeasurementPackage meas_package)
 * 
 * @param meas_package (The latest measurement data of either radar or laser)
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {


  if (is_initialized_) {

    // Compute the elapsed time from the previous measurement/state.
    double dt = double(meas_package.timestamp_ - previous_timestamp_) / 1000000.0; // Seconds 

    // Store current timestamp for next processing step.
    previous_timestamp_ = meas_package.timestamp_;

    double dt_step = 0.05;
    while (dt > 0.1) {
      GetPoints(dt_step);
      dt -= dt_step;
    }
    GetPoints(dt);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR 
        || meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Update
      UpdateState(meas_package);
    } else {
      cout << "Measurement is neither Sensor nor Radar. Skipped." << endl;
    }
  } else {
    // Set timestamp and initialize filter
    previous_timestamp_ = meas_package.timestamp_;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR
        || meas_package.sensor_type_ == MeasurementPackage::LASER ) {     
      Initialize(meas_package);
      is_initialized_ = true;
    } else {
        cout << "Filter not initialized" << endl;
        is_initialized_ = false;
      }    
    }
}

/**
  * Initializes Filter
**/
void UKF::Initialize(MeasurementPackage meas_package) {
    bool is_radar = false; // false for radar, true for lidar
    float yawRate = 0.0; 
    P_.setZero(5,5);
    if ((is_radar && meas_package.raw_measurements_[0]!= 0)
      || (!is_radar && meas_package.raw_measurements_[0] == 0 && meas_package.raw_measurements_[1] == 0)) {
     
       x_ << 0,0,0,0,0;
       is_initialized_ = false;
       return;
    } else if (is_radar){

      float range = meas_package.raw_measurements_[0];
      float bearing = meas_package.raw_measurements_[1];
      float rangeRate = meas_package.raw_measurements_[2]; 
      float px = range * cos(bearing);
      float py = range * sin(bearing);
      float vx = rangeRate * cos(bearing);
      float vy = rangeRate * sin(bearing);
      float yaw = bearing;
      x_ << px, py, rangeRate, yaw, yawRate;

    } else if (!is_radar) {
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      float vMag = 0.0;
      float yaw = 0.0;
      x_ << px, py, vMag, yaw, yawRate;
    }   


    // Populate only the diagonal elements.
    P_(0,0) = std_laspx_ * std_laspx_;
    P_(1,1) = std_laspy_ * std_laspy_;
    P_(2,2) = 0.5;
    P_(3,3) = 0.5;
    P_(4,4) = 0.5;

    is_initialized_ = true;

}
/**
  * UKF::PredictMeanAndCovariance()
  * Predicts the mean and covariance of sigma points
**/
void UKF::PredictMeanAndCovariance() {

  //create vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  
  double weight_0 = lambda_/(lambda_ + n_aug_);
  double weight = 0.5/(n_aug_+lambda_);

  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_ + 1; i++) {
    weights_(i) = weight;
  } 

  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);
  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x = x + weights_(i) * Xsig_pred_.col(i);
  }
 
  //predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x;

    //angle normalization
    x_diff(3) = NormalizeAngle(x_diff(3));

    P = P + (weights_(i) * x_diff * x_diff.transpose()) ;
  }
  // write result
  x_ = x;
  P_ = P;

}


/**
  * UKF::GetPoints(double delta_t)
  * Creates sigma points
  * @param: delta_t (change in time between measurements)
  */
void UKF::GetPoints(double delta_t) {

   // Generate matrix to hold sigma points.
  Xsig_ = MatrixXd(n_aug_, 2*n_aug_+1);

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
 
  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  float sig = sqrt(lambda_ + n_aug_);
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sig * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sig * L.col(i);
  }

  //write result
  Xsig_ = Xsig_aug;

  // Predict Sigma Points
  PredictPoints(delta_t);

  // Predict mean and co-variance
  PredictMeanAndCovariance();
}

/**
  * UKF::PredictPoints(double delta_t)
  * Predicts sigma points
  * @param: delta_t (change in time between measurements)
**/
void UKF::PredictPoints(double delta_t) {

  // re-initialize matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_(0,i);
    double p_y = Xsig_(1,i);
    double v = Xsig_(2,i);
    double yaw = Xsig_(3,i);
    double yawd = Xsig_(4,i);
    double nu_a = Xsig_(5,i);
    double nu_yawdd = Xsig_(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

}

/**
  * UKF::PredictMeasurement
  * Predicts sigma points
  * @param: is_radar(specifies if we are measuring radar or lidar)
**/
void UKF::PredictMeasurement(bool is_radar) {
  // Assume laser, change to 3 if radar
  int n_z = 2;
  if (is_radar) {
    n_z = 3;
  }

  //create matrix for sigma points in measurement space
  Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  if (is_radar) {
    // Measure radar
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

      // extract values for better readibility
      double p_x = Xsig_pred_(0,i);
      double p_y = Xsig_pred_(1,i);
      double v  = Xsig_pred_(2,i);
      double yaw = Xsig_pred_(3,i);
      double v1 = cos(yaw)*v;
      double v2 = sin(yaw)*v;

      // Cache expensive sqrt operation to avoid duplication.
      double rMag = sqrt(p_x*p_x + p_y*p_y);

      // measurement model
      Zsig(0,i) = rMag; //r     - Range
      Zsig(1,i) = atan2(p_y,p_x);  //phi   - Bearing
      if (rMag > 0.00001) {
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / rMag;   //r_dot - Range Rate
      } else {
        Zsig(2,i) = 0;
      }
    }
  } else {
    // Measure lidar
     //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      Zsig(0,i) = Xsig_pred_(0,i);
      Zsig(1,i) = Xsig_pred_(1,i);
    }

  }

  //mean predicted measurement
  z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);

  if (is_radar) {
    R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;

  } else {
    R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
  }

  S = S + R;
}


/**
 * UKF::UpdateState(MeasurementPackage meas_package)
 * Updates the state and the state covariance matrix
 * @params: meas_package (measurement package)
 **/
void UKF::UpdateState(MeasurementPackage meas_package) {

  bool is_radar = (meas_package.sensor_type_ == MeasurementPackage::RADAR);
  int n_z = 2;

  if (is_radar) {
    n_z = 3;
  }
  z_pred = VectorXd(n_z);
  S = MatrixXd(n_z, n_z);
  PredictMeasurement(is_radar);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, z_pred.size());

  //calculate cross correlation matrix
  Tc.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    if (is_radar) {
      //angle normalization (bearing)
      z_diff(1) = NormalizeAngle(z_diff(1));  
      //angle normalization (psi)
      x_diff(3) = NormalizeAngle(x_diff(3));
    }

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  MatrixXd Sinv = S.inverse();
  MatrixXd K = Tc * Sinv;
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization (bearing)
  z_diff(1) = NormalizeAngle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  // Write out the NIS calculation
  if (is_radar) {
    NIS_radar_ = z_diff.transpose()*Sinv*z_diff;
  } else {
    NIS_laser_ = z_diff.transpose()*Sinv*z_diff;
  }
}

  /**
  * UKF::NormalizeAngle(double angle)
  * Wraps the input angle into [-pi pi]
  * @params: angle
  **/
double UKF::NormalizeAngle(double angle) {
  if (angle > M_PI) {
    return remainder(angle, (2.*M_PI)) - M_PI;
  } else if (angle < -M_PI) {
    return remainder(angle, (2.*M_PI)) + M_PI;
  } else {
    return angle;
  }
}
