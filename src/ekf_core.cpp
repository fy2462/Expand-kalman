//
// Created by Feng,Yan on 2018/2/7.
//

#include "../include/ekf_core.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

EKFCore::EKFCore() : is_initialized_(false),
                     previous_timestamp_(0),
                     R_laser_(MatrixXd(2, 2)),
                     R_radar_(MatrixXd(3, 3)),
                     H_laser_(MatrixXd(2, 4)),
                     Hj_(MatrixXd(3, 4)),
                     F_(MatrixXd(4, 4)),
                     Q_(MatrixXd(4, 4)),
                     P_(MatrixXd(4, 4)),
                     x_(VectorXd(4)) {

    R_laser_ << 0.0225, 0,
                0, 0.0225;

    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // P K-1
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

    x_ << 1, 1, 1, 1;

    // initialize variables in Kalman Filter
    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);

    tools = Tools();
}

EKFCore::~EKFCore() {}

void EKFCore::ProcessData(const MeasurementPackage &measurement_pack) {

    // prediction

    long long incoming = measurement_pack.timestamp_;
    float dt = (incoming - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = incoming;

    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    ekf_.Q_ << dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0,
            0, dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_,
            dt_3 / 2 * noise_ax_, 0, dt_2 * noise_ax_, 0,
            0, dt_3 /2 * noise_ay_, 0, dt_2 * noise_ay_;

    ekf_.Predict();

    // update

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // calc jacobian to map to linear
        ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // laser updates
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;

}

void EKFCore::initData(const MeasurementPackage &measurement_pack) {

    if (!is_initialized_) {

        // first measurement
        cout << "EKF: " << endl;
        previous_timestamp_ = measurement_pack.timestamp_;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            //convert from polar to cartesian
            double ro = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            ekf_.x_ << ro*std::cos(phi), ro*std::sin(phi), 0, 0;
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        }

        is_initialized_ = true;
    }
}

KalmanFilter EKFCore::getFilter() {
    return ekf_;
}