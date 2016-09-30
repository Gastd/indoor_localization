#ifndef EKF_H
#define EKF_H

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>


/**
 * @brief Extended Kalman Filter
 * @details The Extended Kalman Filter is a non-linear version of Kalman Filter, this aproach 
 * aproximate the distribution
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

class ekf
{
public:
    ekf();
    ~ekf();

private:
    double dt_;

    void prediction();
    void update();

};

#endif
