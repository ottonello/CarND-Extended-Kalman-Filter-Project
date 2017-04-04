#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {
    simplified = true;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;

    DoUpdate(z, z_pred);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    double px = x_.coeff(0);
    double py = x_.coeff(1);
    double vx = x_.coeff(2);
    double vy = x_.coeff(3);

    double c1 = px * px + py * py;
    if (fabs(c1) < MIN_VALUE) {
        c1 = c1 > 0 ? MIN_VALUE : -MIN_VALUE;
    }

    VectorXd z_pred = VectorXd(3);

    double rho = sqrt(c1);
    double phi = atan2(py, px);
    double rho_dot = (px * vx + py * vy) / rho;
    z_pred << rho, phi, rho_dot;

    DoUpdate(z, z_pred);
}

void KalmanFilter::DoUpdate(const VectorXd &z, const VectorXd &z_pred) {
    VectorXd y = z - z_pred;

    // Readjust y angle values so it's between -pi and +pi
    while (y(1) < -M_PI) {
        y(1) += 2 * M_PI;
    }
    while (y(1) > M_PI) {
        y(1) -= 2 * M_PI;
    }

    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    //new estimate
    x_ = x_ + (K * y);

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    if(simplified){
        P_ = (I - K * H_) * P_;
    } else{
        MatrixXd Kt = K.transpose();
        P_ = P_ - (K * H_ * P_) - (P_ * Ht * Kt) + (K * S * Kt);
    }

}
