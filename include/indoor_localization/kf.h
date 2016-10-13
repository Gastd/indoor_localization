#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "indoor_localization/filter.h"

/**
 * @brief Kalman Filter
 * @details The Kalman Filter is a non-linear version of Kalman Filter, this aproach 
 * aproximate the posterior distribution with a Taylor series of the motion and measurements models
 * @return state of the robot
 * 
 * *prediction step*
 * X  = A*xk-1 + B*uk-1
 * p = A*P*A.t + Q
 * *update step*
 * K = p*H.t*(H*p*H.t + R)^-1
 * xk = X + K*(z - H*X)
 * Pxk = (I - K*H)*p
 */

class KF: public Filter
{
public:
    KF();

    Eigen::MatrixXd getState();

    ~KF();

protected:
    Eigen::VectorXd state_, control_;
    virtual void initFilter();

private:
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd state_model_, sensor_model_;
    Eigen::MatrixXd kalman_gain_;
    Eigen::MatrixXd process_noise_covariance_, sensor_covariance_;

    void predict();
    void update();
};

#endif
