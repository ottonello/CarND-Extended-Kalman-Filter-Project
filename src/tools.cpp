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
    /**
    TODO:
      * Calculate a Jacobian here.
    */
    MatrixXd Hj(3,4);
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
    if(fabs(c1) < 0.0001){
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    //compute the Jacobian matrix
    Hj << (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return Hj;

//    MatrixXd Hj(3, 4);
//    //recover state parameters
//    double px = x_state(0);
//    double py = x_state(1);
//    double vx = x_state(2);
//    double vy = x_state(3);
//
//    //check division by zero
//    if (px == 0 && py == 0) {
//        std::cout << "px and py are both zero, will result in division by zero" << std::endl;
//        px = std::nextafter(0, 1.0f);
//        py = std::nextafter(0, 1.0f);
////        return Hj;
//    }
//    //compute the Jacobian matrix
//    double px2 = px * px;
//    double py2 = py * py;
//    double sum = px2 + py2;
//    double quadrSum = std::sqrt(sum);
//    Hj(0, 0) = px / quadrSum;
//    Hj(0, 1) = py / quadrSum;
//    Hj(1, 0) = -py / (sum);
//    Hj(1, 1) = px / (sum);
//    Hj(2, 0) = py * (vx * py - vy * px) / std::pow(sum, 3 / 2);
//    Hj(2, 1) = px * (vy * px - vx * py) / std::pow(sum, 3 / 2);
//    Hj(2, 2) = px / quadrSum;
//    Hj(2, 3) = py / quadrSum;
//    return Hj;
}
