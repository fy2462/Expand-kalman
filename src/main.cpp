//
// Created by Feng,Yan on 2018/2/7.
//

#include <iostream>
#include <sstream>
#include <fstream>
#include "../include/common/BundleParams.h"
#include "../include/data_package.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void check_files(ifstream &in_file, string &in_name,
                 ofstream &out_file, string &out_name) {
    if (!in_file.is_open()) {
        cerr << "Cannot open input file: " << in_name << endl;
        exit(EXIT_FAILURE);
    }

    if (!out_file.is_open()) {
        cerr << "Cannot open output file: " << out_name << endl;
        exit(EXIT_FAILURE);
    }
}

void getDataPackage(ifstream& in_file_, vector<MeasurementPackage>& measurement_pack_list, vector<GroundTruthPackage>& gt_pack_list){

    string line;

    while (getline(in_file_, line)) {

        string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        long long time_stamp;

        istringstream iss(line);

        iss >> sensor_type;

        if (sensor_type.compare("L") == 0) {
            // LASER MEASUREMENT
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> time_stamp;
            meas_package.timestamp_ = time_stamp;
            measurement_pack_list.push_back(meas_package);
        } else if (sensor_type.compare("R") == 0) {
            // RADAR MEASUREMENT
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> time_stamp;
            meas_package.timestamp_ = time_stamp;
            measurement_pack_list.push_back(meas_package);
        }

        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);

    }

}

void process_EKF(ifstream& in_file_, ofstream& out_file_, BundleParams& params) {

    vector<MeasurementPackage> measurement_pack_list;
    vector<GroundTruthPackage> gt_pack_list;

    getDataPackage(in_file_, measurement_pack_list, gt_pack_list);




}

int main(int argc, char **argv) {
    BundleParams params(argc, argv);  // set the parameters here.

    if (params.input.empty()) {
        std::cout << "Usage: ExtendedKF -input <path for dataset> -output <path for output file>";
        return 1;
    }

    string in_file_name_ = params.input;
    ifstream in_file_(in_file_name_.c_str(), ifstream::in);

    string out_file_name_ = params.output;
    ofstream out_file_(out_file_name_.c_str(), ofstream::out);

    check_files(in_file_, in_file_name_, out_file_, out_file_name_);

    process_EKF(in_file_, out_file_, params);

    if (out_file_.is_open()) {
        out_file_.close();
    }

    if (in_file_.is_open()) {
        in_file_.close();
    }

    return 0;
}

