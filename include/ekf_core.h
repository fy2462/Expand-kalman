//
// Created by Feng,Yan on 2018/2/7.
//

#ifndef EXTENDEDKF_EKF_CORE_H
#define EXTENDEDKF_EKF_CORE_H

#include "../include/data_package.h"

class EKFCore {

    EKFCore();
    virtual ~EKFCore();

    void ProcessData(const MeasurementPackage &measurement_pack);

private:

    bool is_initialized_;

    long long previous_timestamp_;

    Tools tools;

    KalmanFilter ekf_;

    // laser
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd H_laser_;
    // radar
    Eigen::MatrixXd R_radar_;
    // x vector jacobian matrix
    Eigen::MatrixXd Hj_;

    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd P_;
    VectorXd x_;


    int noise_ax_ = 3 * 3;
    int noise_ay_ = 3 * 3;
};


#endif //EXTENDEDKF_EKF_CORE_H
