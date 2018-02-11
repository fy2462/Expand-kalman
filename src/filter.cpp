//
// Created by Feng,Yan on 2018/2/7.
//

#include "../include/filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &StateTrans_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &AccCovariance_in) {
    x_ = x_in;
    P_ = P_in;
    StateTrans_ = StateTrans_in;
    H_ = H_in;
    R_ = R_in;
    AccCovariance_ = AccCovariance_in;
}

void KalmanFilter::Predict() {
    x_ = StateTrans_ * x_;
    P_ = StateTrans_ * P_ * StateTrans_.transpose() + AccCovariance_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd y = VectorXd(2);
    y = z - H_ * x_;

    MatrixXd PHt = P_ * H_.transpose();

    MatrixXd S = MatrixXd(2, 2);
    S = H_ * PHt + R_;

    MatrixXd K = MatrixXd(2, 2);
    K = PHt * S.inverse();

    x_ = x_ + K * y;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

// same equations but with jacobian Hj and h(x) conversion to polar
void KalmanFilter::UpdateEKF(const VectorXd &z) {
    float sqrt_sq = std::sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    // prevent 0 vals as atan2(0,0) undefined
    if (fabs(x_(0)) < 1e-6)
        x_(0) = 1e-6;
    if (fabs(x_(1)) < 1e-6)
        x_(1) = 1e-6;

    VectorXd hx(3);
    // use atan2 to keep between -pi and pi
    // non-linear conversion: distance + thet + Radial speed
    hx << sqrt_sq, atan2(x_(1), x_(0)), (x_(0) * x_(2) + x_(1) * x_(3)) / sqrt_sq;

    VectorXd y = z - hx;

    // normalize to keep between -pi and pi
    while (y(1) > 3.1415) {
        y(1) -= 6.2830;
    }
    while (y(1) < -3.1415) {
        y(1) += 6.2830;
    }

    MatrixXd PHt = P_ * H_.transpose();
    MatrixXd S = H_ * PHt + R_;
    MatrixXd K = PHt * S.inverse();
    x_ = x_ + K * y;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}