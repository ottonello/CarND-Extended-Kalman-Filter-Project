#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    /**
    TODO:
      * Calculate the RMSE here.
    */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        return rmse;
    }

    for (int i = 0; i < estimations.size(); ++i) {
        // ... your code here
        VectorXd res = estimations[i] - ground_truth[i];
        res = (res.array() * res.array());
        rmse += res;
    }

    //calculate the mean
    rmse = rmse / estimations.size();
    //calculate the squared root
    rmse = rmse.array().sqrt();
    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
    MatrixXd Hj = MatrixXd::Zero(3, 4);
    //recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    //pre-compute a set of terms to avoid repeated calculation
    double c1 = px*px+py*py;
    double c2 = sqrt(c1);
    double c3 = (c1*c2);

    //check division by zero
    if(fabs(c1) < 0.00001){
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;
}
