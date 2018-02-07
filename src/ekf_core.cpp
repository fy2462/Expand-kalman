//
// Created by Feng,Yan on 2018/2/7.
//

#include "../include/ekf_core.h"

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