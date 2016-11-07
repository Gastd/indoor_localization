#ifndef EKF_H
#define EKF_H

#include <list>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// #include <image_transport/image_transport.h>

#include <Eigen/Dense>

#include "indoor_localization/kf.h"

/**
 * @brief Extended Kalman Filter
 * @details The Extended Kalman Filter is a non-linear version of Kalman Filter, this aproach 
 * aproximate the posterior distribution with a Taylor series of the motion and measurements models
 * @return state of the robot
 * 
 * *prediction step*
 * p = g(ut,xt-1)
 * Covpt = Gt*Covt-1*Gt.t + Rt
 * *correction step*
 * Kt = Covpt*Ht.t*(Ht*Covpt*Ht.t + Qt)^-1
 * xt = p + Kt*(zt - h(p))
 * Covt = (I - Kt*Ht)*Covpt
 *  return xt,Covt
 */

class EKF
{
public:
    EKF();

    void run();
    Eigen::VectorXd getState();
    Eigen::MatrixXd getCovariance();

    ~EKF();
protected:
    void initFilter();
    void initRos();
    void calculateJacobians(Eigen::Vector2d control);
    void publishData();
    void predict(Eigen::Vector2d control);
    void correct(geometry_msgs::PoseWithCovariance measurement);

    void controlCallback(const nav_msgs::OdometryConstPtr& msg);
    void measurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

private:
    /*ROS*/
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    ros::Subscriber control_sub_, measur_sub_;
    ros::Publisher odom_filter_pub_;
    tf::TransformBroadcaster br_;
    tf::TransformListener listener_;
    std::string control_topic_, measur_topic_;

    double dt_;

    /*control[0] = linear velocity, control[1] = angular velocity, control[2] = dt*/
    // Eigen::VectorXd controls_;
    std::list<Eigen::Vector3d> controls_;

    // geometry_msgs::PoseWithCovariance measurements_;
    std::list<geometry_msgs::PoseWithCovariance> measurements_;

    /*State = (x; y; theta)*/
    Eigen::Vector3d state_, state1_;
    Eigen::Matrix3d cov_state_, process_noise_covar_;

    /*Models Jacobians*/
    Eigen::MatrixXd motion_jacob_, motion_noise_jacob_, measurement_jacob_;

    /*Scale parameters*/
    double alpha1_,alpha2_,alpha3_,alpha4_;

    /*cosine and sine of yaw*/
    double cy_, cyw_, sy_, syw_;

    /*covariances*/
    Eigen::Matrix2d motion_noise_;
};

#endif
