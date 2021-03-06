#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2); //AS measurement noise covariance
  R_radar_ = MatrixXd(3, 3); //AS measurement noise covariance
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  //VectorXd x;	// object state
  //MatrixXd P;	// object covariance matrix
  //VectorXd u;	// external motion
  //MatrixXd F; // state transition matrix
  //MatrixXd H;	// measurement matrix
  //MatrixXd R;	// measurement covariance matrix
  //MatrixXd I; // Identity matrix
  //MatrixXd Q;	// process covariance matrix

  //create a 4D state vector, we don't know yet the values of the x state
	VectorXd x = VectorXd(4);

 	//state covariance matrix P
	MatrixXd P = MatrixXd(4, 4);
	P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;

  //the initial transition matrix F_
	MatrixXd F = MatrixXd(4, 4);
	F << 1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // measurement matrix
  MatrixXd H = MatrixXd(2, 4);
	H << 1, 0, 0, 0,
       0, 1, 0, 0;

  // process covariance matrix
  MatrixXd Q = MatrixXd(4, 4);
  Q << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  ekf_.Init(x, P, F, H, R_laser_, Q);

  // measurement covariance matrix
  //ekf_.R_ = MatrixXd(2, 2);
	//ekf_.R_ << 0.0225, 0,
  //           0, 0.0225;


  // Set process noise. u (nu) is variable for process noise. x' = F * x + u
  u = VectorXd(4); // 4 elements of vector are [u_px, u_py, u_vx, u_vy]


  // Set measurement noise. Omega or w is the variable for measurement noise
  // Measurement noise is a 3-element vector for radars and a 2-element vector for lidar

  noise_ax = 9;
	noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */


      //Reference below from main.cpp
      float rho = measurement_pack.raw_measurements_(0); //Range
      float phi = measurement_pack.raw_measurements_(1); //Bearing (relative heading)
      float rho_dot = measurement_pack.raw_measurements_(2); //Range rate

      float cos_phi = cos(phi);
      float sin_phi = sin(phi);

      float px = rho * cos_phi; //x coordinates along vehicle longitudinal axis
      float py = rho * sin_phi;
      float vx = rho_dot * cos_phi;
      float vy = rho_dot * sin_phi;

      ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0; //AS
    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // Compute elapsed time delta_t
  //compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;

	if ( dt > 0.001 ) {
    float dt_2 = dt * dt;
    float dt_3 = dt_2 * dt;
    float dt_4 = dt_3 * dt;

    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // Use delta_t to compute new F and Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
                0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
                0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    ekf_.Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    ekf_.R_ = R_radar_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates

    ekf_.R_ = R_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl << endl;
  cout << "P_ = " << ekf_.P_ << endl << endl;
}
