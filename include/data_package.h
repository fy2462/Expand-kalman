//
// Created by Feng,Yan on 2018/2/7.
//

#ifndef EXTENDEDKF_DATA_PACKAGE_H
#define EXTENDEDKF_DATA_PACKAGE_H

#include "Eigen/Dense"

class MeasurementPackage {
public:
    long long timestamp_;

    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;
};

class GroundTruthPackage {
public:
    long long timestamp_;

    enum SensorType{
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd gt_values_;

};

#endif //EXTENDEDKF_DATA_PACKAGE_H
