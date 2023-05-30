#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools tools;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  H_jacobian = MatrixXd(3, 4);

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

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

	// initialize the kalman filter variables
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // set measurement noises
  noise_ax = 9;
  noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  // 如果没有初始化过，进行初始化状态，传感器作为初始状态
  if (!is_initialized_) {

    // x_为状态量
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      //  将雷达从极地坐标(r, θ)转换为笛卡尔坐标系(x, y)，并初始化状态。
      float rho = measurement_pack.raw_measurements_[0];      // range: radial distance from origin
      float phi = measurement_pack.raw_measurements_[1];      // bearing: angle between rho and x axis
      float rho_dot = measurement_pack.raw_measurements_[2];  // radial velocity: change of rho

      // ekf_.x_ << rho * cos(phi), rho * sin(phi), rho_dot * cos(phi), rho_dot * sin(phi);
      // phi is not the direction of the speed, it is better to set vx and vy to 0
      ekf_.x_ << rho * cos(phi), rho * sin(phi), 0, 0;  // x, y, vx, vy


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
     
      // lidar的值不用经过转换
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0; // x, y, vx, vy
    }
    previous_timestamp_ = measurement_pack.timestamp_;

    // 第一次只进行初始化
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  // compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;  //  时间转化为秒
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // 过程噪声协方差矩阵：对称矩阵
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
             0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
             dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
             0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;
  // 状态预测
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     根据传感器类型选择更新方式
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // 计算雅可比矩阵进行线性化
    H_jacobian = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = H_jacobian;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
}
