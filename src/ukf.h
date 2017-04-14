#ifndef UKF_H
#define UKF_H
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"
#include "measurement_package.h"
#include "Eigen/Dense"


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

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

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

  //* Time of previous measurement 
  double previous_timestamp_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;


  ///* Weights of sigma points
  VectorXd weights_;

  //* Z prediction
  VectorXd z_pred;

  //* X points matrix
  MatrixXd Xsig_; 

  //* Sigma points matrix
  MatrixXd Zsig;

  //* Covariance S matrix
  MatrixXd S;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * Runs the flow of the UKF
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Initialize
   * Initializes values for measurement package
   */

  void Initialize(MeasurementPackage meas_package);

  /**
  * PredictMeasurement
  * Predict measurements for Laser and Radar 
  **/
  void PredictMeasurement(bool is_radar); 

  /**
   * UpdateState
   * Updates the state and the state covariance matrix
   */
  void UpdateState(MeasurementPackage meas_package);

  /**
  * GetPoints
  * Computes the mean and covariance of sigma points
  **/
  void GetPoints(double delta_t);

/**
  * PredictPoints
  * Predicts Sigma Points
  **/
  void PredictPoints(double delta_t);

  /**
  * PredictMeanAndCovariance
  * Computes the mean and covariance of sigma points
  **/
  void PredictMeanAndCovariance();
  
  /**
  * NormalizeAngle
  * Wraps the input angle into [-pi pi]
  **/
  double NormalizeAngle(double angle);

};

#endif /* UKF_H */
