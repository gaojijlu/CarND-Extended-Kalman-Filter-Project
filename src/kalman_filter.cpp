#include "kalman_filter.h"
#include <iostream>
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {

}

KalmanFilter::~KalmanFilter() {

}

void KalmanFilter::InitializeKF(const Eigen::VectorXd &x_init, const Eigen::MatrixXd &P_init,
                                const Eigen::MatrixXd &Q_init, const Eigen::MatrixXd &F_init,
                                const Eigen::MatrixXd &H_init, const Eigen::MatrixXd &R_init){
    x_ = x_init;
    P_ = P_init;
    Q_ = Q_init;
    F_ = F_init;
    H_ = H_init;
    R_ = R_init;
}


void KalmanFilter::Predict(){
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateKF(const VectorXd &z){
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;

    x_ = x_ + K * y;
    MatrixXd I = MatrixXd::Identity(x_.size(),x_.size());
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z){
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    if ((px * px + py * py) < 0.0001 || fabs(px) < 0.0001){
        cout << "Error-Division by zero" << endl;
        return;
    }

    VectorXd hx(3);
    hx << sqrt(px*px+py*py), atan2(py, px), (px * vx + py * vy)/sqrt(px * px + py * py);
    hx(1) = atan2(sin(hx(1)), cos(hx(1))); // not sure if it is right

    VectorXd y = z - hx;
    MatrixXd Hj = H_;
    MatrixXd Hjt = Hj.transpose();
    MatrixXd PHjt = P_ * Hjt;
    MatrixXd S = H_ * PHjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHjt * Si;

    x_ = x_ + K * y;
    MatrixXd Id = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (Id - K * Hj) * P_;
    
}

