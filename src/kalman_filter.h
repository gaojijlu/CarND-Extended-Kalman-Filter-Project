#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter {

public:
    // state vector
    VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // process covariance matrix
    MatrixXd Q_;

    // state transition matrix
    MatrixXd F_;    

    // measuremnt matrix
    MatrixXd H_;

    // measurement covariance matrix
    MatrixXd R_;

    KalmanFilter();
    virtual ~KalmanFilter();

    /**
     * @brief Initializes Kalman Filter
     * @param x_init Initial state
     * @param P_init Initial state covariance
     * @param Q_init Process covariance matrix
     * @param F_init Transition matrix
     * @param H_init Measuremnt matrix
     * @param R_init Measurement covariance matrix
     */
    void InitializeKF(const VectorXd &x_init, const MatrixXd &P_init,
                      const MatrixXd &Q_init, const MatrixXd &F_init,
                      const MatrixXd &H_init, const MatrixXd &R_init);

    /**
     * @brief Predict state and state covariance using the process model
     * @param delta_t Time between k and k+1 in seconds
     */
    void Predict();

    /**
     * @brief Update the state and state covariance using the standard kalman filter
     * equation with the measuremnt at k+1
     * @param z Measurement at time k+1
     */
    void UpdateKF(const VectorXd &z);


    /**
     * @brief Update the state and state covariance using the extended kalman filter
     * equation with the measuremnt at k+1
     * @param z Measurement at time k+1
     */
    void UpdateEKF(const VectorXd &z);
};

#endif /* KALMAN_FILTER_H_ */
