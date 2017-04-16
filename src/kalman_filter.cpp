#include "kalman_filter.h"
#include "tools.h" //AS
#include <iostream> //AS

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std; //AS

KalmanFilter::KalmanFilter() {
  //PI = std::atan(1.0)*4; //AS
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */

  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  Tools t;

  MatrixXd Hj_ = t.CalculateJacobian(x_);
  VectorXd h(3);

  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float rho = sqrt(pow(px, 2) + pow(py, 2));

  float phi = atan2(py, px);
  if (fabs(px) < 0.001)
    phi = atan2(py, 0.001); //phi = atan2(0.0001, 0.001);

  float rho_dot = 0.0;
  if (fabs(rho) > 0.001)
    rho_dot = (px*vx + py*vy)/rho;

  h << rho, phi, rho_dot;

  cout << "px " << px << " py " << py << " rho " << rho << " phi " << phi << " rho_dot " << rho_dot << endl << endl;

	VectorXd y = z - h; // measurement - prediction in polar coordinates
	cout << "y " << y << endl << endl;

  // Adjust second element of y (bearing angle phi) until it is between -pi and pi
  while(y[1] < -PI)
    y[1] += 2 * PI;
  while(y[1] > PI)
    y[1] -= 2 * PI;

	MatrixXd Hjt = Hj_.transpose();
	MatrixXd S = Hj_ * P_ * Hjt + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHjt = P_ * Hjt;
	MatrixXd K = PHjt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;
}
