#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {

public:
    // kalman filter instance,
    KalmanFilter kalman_filter_;

    // implicit iniatialize R_laser_, R_radar_, H_laser_ and
    // some members of kalman_filter_
    FusionEKF();
    virtual ~FusionEKF();

    /**
     * @brief ProcessKalmanFilterFlow: Process the whole work flow of Kalman Filter from here,
     * including 3 steps: Initialization, Prediction, Update
     * @param measurement_pack: measurement raw data
     */
    void ProcessKalmanFilterFlow(const MeasurementPackage &measurement_pack);

private:
    // check whether the tracking toolbox was initialized or not (first measurement)
    bool is_initialized_;

    // previous timestamp
    long long previous_timestamp_;

    // tool object used to compute Jacobian and RMSE
    Tools tools_;

    // these will be assigned to kalman_filter_ 's members when figure out
    Eigen::MatrixXd R_laser_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_laser_;
    Eigen::MatrixXd Hj_;
};

#endif /* FusionEKF_H_ */
