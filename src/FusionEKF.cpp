#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


FusionEKF::FusionEKF() {

    is_initialized_ = false;
    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    //measurement covariance matrix - laser(provide by laser maker)
    R_laser_ << 0.0225, 0,
            0, 0.0225;
    //measurement covariance matrix - radar(provide by radar maker)
    R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;
    // measurement matrix - laser
    H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

    // initializing members of kalman_filter_
    kalman_filter_.x_ = VectorXd(4);
    kalman_filter_.P_ = MatrixXd(4, 4);
    kalman_filter_.F_ = MatrixXd(4, 4);
    kalman_filter_.Q_ = MatrixXd(4, 4);
    // do not set to all zeros
    kalman_filter_.x_ << 1, 1, 1, 1;
    // why 1000?
    kalman_filter_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
    // partial initialize elements
    kalman_filter_.F_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // initialize the size of Jacobian Matrix with no init value
    Hj_ = MatrixXd(3, 4);

}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessKalmanFilterFlow(const MeasurementPackage &measurement_pack) {
    /*****************************************************************************
    * 1. Initialization
    * Initialize the state kalman_filter_.x_ with the first measurement.
    ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        cout << "First measurement " << endl;
        previous_timestamp_ = measurement_pack.timestamp_;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = measurement_pack.raw_measurements_[0];
            double phi = measurement_pack.raw_measurements_[1];
            // initialize state by converting radar data from polar to cartesian coordinates
            kalman_filter_.x_ << rho * sin(phi), rho * cos(phi), 0, 0;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            kalman_filter_.x_ << measurement_pack.raw_measurements_[0],
                    measurement_pack.raw_measurements_[1], 0, 0;
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        // skip process below
        return;
    }

    /*****************************************************************************
    * 2. Prediction
    * Update the state transition matrix kalman_filter_.F_ and process covariance
    * matrix kalman_filter_.Q_ according to the new elapsed time.
    ****************************************************************************/
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    if (dt < 0.0001)
        dt = 0.0001;
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    // complete transition matrix F_
    kalman_filter_.F_(0, 2) = dt;
    kalman_filter_.F_(1, 3) = dt;

    // process noise covariance matrix
    float noise_ax = 9.0;
    float noise_ay = 9.0;
    kalman_filter_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    // call predict function
    kalman_filter_.Predict();


    /*****************************************************************************
    * 3. Update
    * Perform the corresponding update steps according to the sensor type
    ****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        Hj_ = tools_.CalculateJacobian(kalman_filter_.x_);
        kalman_filter_.H_ = Hj_;
        kalman_filter_.R_ = R_radar_;
        kalman_filter_.UpdateEKF(measurement_pack.raw_measurements_);

    } else {
        // Laser updates
        kalman_filter_.H_ = H_laser_;
        kalman_filter_.R_ = R_laser_;
        kalman_filter_.UpdateKF(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << kalman_filter_.x_ << endl;
    cout << "P_ = " << kalman_filter_.P_ << endl;

}
