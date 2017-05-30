#include "indoor_localization/ekf.h"


//-------------- Utility functions -------------------------
inline static double range(double dx, double dy)
{
    return sqrt(dx*dx+dy*dy);
}

inline static double bearing(double dx, double dy/*, double theta*/)
{
    // return angles::normalize_angle(atan2(dy,dx)/*-theta*/);
    return atan2(dy,dx)/*-theta*/;
}
//-------------- Utility functions -------------------------

EKF::EKF()
{
    initFilter();
    initRos();
}

EKF::~EKF() {}

void EKF::initRos()
{
    /*Initalize Ros*/
    control_topic_ = "pose";
    measur_topic_ = "ar_pose_marker";
    ros::NodeHandle node_handle_;
    control_sub_  = node_handle_.subscribe(control_topic_, 1, &EKF::controlCallback, this);
    measur_sub_ = node_handle_.subscribe(measur_topic_, 1, &EKF::measurementCallback, this);
    // odom_filter_pub_ = node_handle_.advertise<nav_msgs::Odometry>("indoor_ekf/odometry/filtered", 0);
    odom_filter_pub_ = node_handle_.advertise<geometry_msgs::PoseWithCovarianceStamped>("indoor_ekf/pose", 0);

    bool tfCatched = false;
    while(!tfCatched)
    {
        tfCatched = true;
        try
        {
            listener_.lookupTransform ("base_link", "usb_camera_optical_frame", ros::Time(0), baseToCamStamped_);
        }
        catch (tf::TransformException ex)
        {
            // ROS_ERROR("%s", ex.what());
            tfCatched = false;
        }
    }
}

void EKF::controlCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    static ros::Time old_time;
    ros::Duration dt = odom_msg->header.stamp - old_time;
    // ROS_INFO_STREAM("odom stamp " << odom_msg->header.stamp << " old_time; " << old_time);
    old_time = odom_msg->header.stamp;

    // ROS_INFO_STREAM("dt " << dt );
    if(dt.toSec() > 1)
        return;

    double v = odom_msg->twist.twist.linear.x;
    double w = odom_msg->twist.twist.angular.z;

    // controls_.push_back();
    dt_ = dt.toSec();
    // ROS_INFO_STREAM("v = " << v);
    // ROS_INFO_STREAM("w = " << w);
    // ROS_INFO_STREAM("dt = " << dt_);
    predict(Eigen::Vector2d(v,w));
}

// void EKF::measurementCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
// {
//     // measurements_.push_back(pose_msg->pose);
//     correct(pose_msg->pose);
// }

void EKF::measurementCallback(const ar_pose::ARMarkers::ConstPtr& pose_msg)
{
    // measurements_.push_back(pose_msg->pose);
    // correct(pose_msg->pose);
    Eigen::Vector2d measurement, state_measurement;
    tf::Point marker_position_camera_frame, marker_position_base_frame;
    // geometry_msgs::Point marker_position_camera_frame = pose_msg->markers.pose.pose.position;
    std::vector<ar_pose::ARMarker> markers = pose_msg->markers;

    if(markers.size() > 0)
    {
        for(auto marker_it = markers.begin(); marker_it != markers.end(); ++marker_it)
        {
            tf::pointMsgToTF(marker_it->pose.pose.position, marker_position_camera_frame);

            marker_position_base_frame = baseToCamStamped_ * marker_position_camera_frame;
            // ROS_INFO_STREAM("---------------- measurement -----------------");
            // ROS_INFO_STREAM("X = " << marker_position_base_frame.getX());
            // ROS_INFO_STREAM("Y = " << marker_position_base_frame.getY());
            // ROS_INFO_STREAM("Z = " << marker_position_base_frame.getZ() << std::endl);
            double dx = marker_position_base_frame.getX();
            double dy = marker_position_base_frame.getY();
            measurement(0) = bearing(dx,dy);  // bearing
            measurement(1) = range(dx,dy);  // range
            correct(marker_it->id, measurement);
        }
    }
}

void EKF::initFilter()
{
    /*Initialize state*/
    state_ << 0, 0, 0;
    cov_state_ = Eigen::MatrixXd::Identity(3,3);
    cov_state_ *= 1e-2;

    /*Initialize variances*/
    alpha1_ = 1e-2;
    alpha2_ = 1e-2;
    alpha3_ = 1e-2;
    alpha4_ = 1e-2;
    motion_noise_ = Eigen::MatrixXd::Zero(2,2);

    // Kalman filter Rt, 
    double std_x = 1e-1;
    double std_y = 1e-1;
    double std_yaw = 1e-1;
    process_noise_covar_ << std_x*std_x, 0, 0,
                            0, std_y*std_y, 0,
                            0,0,std_yaw*std_yaw;

    /*Initialize Jacobians*/
    motion_jacob_ = Eigen::MatrixXd::Zero(3,3);
    motion_noise_jacob_ = Eigen::MatrixXd::Zero(3,2);
    measurement_jacob_ = Eigen::MatrixXd::Zero(3,3);

    cy_ = cyw_ = 0;
    sy_ = syw_ = 0;
}

void EKF::calculateJacobians(Eigen::Vector2d control)
{
    double v = control(0);
    double w = control(1);
    double yaw = state_(2);

    cy_ = cyw_ = 0;
    sy_ = syw_ = 0;
    ::sincos(yaw, &sy_, &cy_);
    ::sincos((yaw + w*dt_), &syw_, &cyw_);

    motion_jacob_(0,0) = 1;
    motion_jacob_(1,0) = 0;
    motion_jacob_(2,0) = 0;

    motion_jacob_(0,1) = 0;
    motion_jacob_(1,1) = 1;
    motion_jacob_(2,1) = 0;

    motion_jacob_(2,2) =  1;

    motion_noise_jacob_(2,0) = 0;

    motion_noise_jacob_(2,1) = dt_;

    motion_noise_(0,0) = 1e-3;
    motion_noise_(1,0) = 0;
    motion_noise_(0,1) = 0;
    motion_noise_(1,1) = 1e-2;

    measurement_jacob_(0,0) = 1;
    measurement_jacob_(1,0) = 0;
    measurement_jacob_(2,0) = 0;

    measurement_jacob_(0,1) = 0;
    measurement_jacob_(1,1) = 1;
    measurement_jacob_(2,1) = 0;

    measurement_jacob_(0,2) = 0;
    measurement_jacob_(1,2) = 0;
    measurement_jacob_(2,2) = 1;

    // Deal with singularities in v_w using L'Hopital's Rule
    if((fabs(w) > 1e-5))
    {
        double v_w = v/w;
        motion_jacob_(0,2) = -v_w*cos(yaw) + v_w*cos(yaw + w*dt_);
        motion_jacob_(1,2) = -v_w*sin(yaw) + v_w*sin(yaw + w*dt_);

        motion_noise_jacob_(0,0) = (-sy_ + syw_)/w;
        motion_noise_jacob_(1,0) = ( cy_ - cyw_)/w;
        motion_noise_jacob_(0,1) =  v*(sy_ - syw_)/(w*w) + v*cyw_*dt_/w;
        motion_noise_jacob_(1,1) = -v*(cy_ - cyw_)/(w*w) + v*syw_*dt_/w;
    }
    else
    {
        motion_jacob_(0,2) = -v*sy_*dt_;
        motion_jacob_(1,2) =  v*cy_*dt_;

        motion_noise_jacob_(0,0) =  cy_*dt_;
        motion_noise_jacob_(0,0) =  sy_*dt_;
        motion_noise_jacob_(0,1) = -(dt_*dt_)*v*sy_/2.;
        motion_noise_jacob_(1,1) =  (dt_*dt_)*v*cy_/2.;
    }
    ROS_DEBUG_STREAM("------ Motion Jacobian -------");
    ROS_DEBUG_STREAM("\n" << motion_noise_jacob_);
}

void EKF::predict(Eigen::Vector2d control)
{
    // Eigen::VectorXd control = controls_.front();
    double v   = control(0);
    double w   = control(1);
    double yaw = state_(2);
    calculateJacobians(control);

    Eigen::Vector3d mean, delta_state;

    // Deal with singularity in v_w using L'Hopital's Rule
    if((fabs(w) > 1e-5))
    {
        double v_w = v/w;
        delta_state(0) = -v_w*sin(yaw) + v_w*sin(yaw + w*dt_);
        delta_state(1) =  v_w*cos(yaw) - v_w*cos(yaw + w*dt_);
        delta_state(2) =  w*dt_;
    }
    else
    {
        delta_state(0) =  v*cy_*dt_;
        delta_state(1) =  v*sy_*dt_;
        delta_state(2) =  0;
    }

    // p = g(ut,xt-1)
    state1_ = state_;
    state_ += delta_state;

    // Covpt = Gt*Covt-1*Gt.t + Rt
    Eigen::MatrixXd prediction_noise = motion_noise_jacob_ * motion_noise_ * motion_noise_jacob_.transpose();
    cov_state_ = motion_jacob_ * cov_state_ * motion_jacob_.transpose() + prediction_noise;

    v = (state_(0) - state1_(0))*cos(state_(2))/dt_ + (state_(1) - state1_(1))*sin(state_(2))/dt_;
    // ROS_INFO_STREAM("v = " << v);
    w = (state_(2) - state1_(2))/dt_;
    // ROS_INFO_STREAM("w = " << w);
    // ROS_INFO("------- Predict State ----------");
    // ROS_INFO_STREAM("predict state: \n" << state_);
    // ROS_INFO_STREAM("\n" << cov_state_);
}

void EKF::correct(uint32_t measurement_id, Eigen::Vector2d measurement)
{
    // Kt = Covpt*Ht.t*(Ht*Covpt*Ht.t + Qt)^-1
    // xt = p + Kt*(zt - h(p))
    // Covt = (I - Kt*Ht)*Covpt

    Eigen::Vector2d state_measurement;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
    std::vector<double> m = map_.get(measurement_id);
    double dx = state_(0) - m[0];
    double dy = state_(1) - m[1];
    state_measurement(0) = bearing(dx,dy);
    state_measurement(1) = range(dx,dy);

    // state_measurement = state_;

    // measurement(0) = measurement_msg.pose.position.x;
    // measurement(1) = measurement_msg.pose.position.y;
    // measurement(2) = tf::getYaw(measurement_msg.pose.orientation);

    // // measurement test, euclidean test
    // if( fabs(state_(0) - measurement(0)) > 1. || 
    //     fabs(state_(1) - measurement(1)) > 1. ||
    //     fabs(state_(2) - measurement(2)) > 1.  )
    //     return;

    Eigen::MatrixXd hcovhq = measurement_jacob_ * cov_state_ * measurement_jacob_.transpose() + process_noise_covar_;
    Eigen::MatrixXd kalman_gain = cov_state_ * measurement_jacob_.transpose() * hcovhq.inverse();

    state_ += kalman_gain * (measurement - state_measurement);
    cov_state_ = (I - kalman_gain * measurement_jacob_)*cov_state_;
    ROS_INFO("------- Measurement State ----------");
    // ROS_INFO_STREAM("measurement: \n" << measurement);
    // ROS_INFO_STREAM("corrected State: \n" << state_);
    // ROS_INFO_STREAM("\n" << cov_state_);
}

void EKF::run()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        publishData();
    }
}

void EKF::publishData()
{
    geometry_msgs::PoseWithCovarianceStamped odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "map";
    // odom.child_frame_id = "base_link";
    
    odom.pose.pose.position.x = state_(0);
    odom.pose.pose.position.y = state_(1);
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_(2));
    odom.pose.covariance[0]  = cov_state_(0); // x x
    odom.pose.covariance[1]  = cov_state_(1); // x y
    odom.pose.covariance[5]  = cov_state_(2); // x yaw

    odom.pose.covariance[6]  = cov_state_(3); // x y
    odom.pose.covariance[7]  = cov_state_(4); // y y
    odom.pose.covariance[11] = cov_state_(5); // y yaw

    odom.pose.covariance[30] = cov_state_(6); // x yaw
    odom.pose.covariance[31] = cov_state_(7); // y yaw
    odom.pose.covariance[35] = cov_state_(8); // yaw yaw


    // The localization node should send map_frame to odom_frame transform,
    // the odom_frame to base_frame is send by p2os_driver, this node estimate
    // the transform between map_frame and base_frame, which has been find above
    tf::Transform base_to_map;
    tf::StampedTransform odom_to_base;
    tf::Quaternion rotation;

    try
    {
        listener_.lookupTransform("odom", "base_link", ros::Time(0), odom_to_base);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }
    tf::quaternionMsgToTF(odom.pose.pose.orientation, rotation);
    base_to_map.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0) );
    base_to_map.setRotation(rotation);

    tf::Transform odom_to_map = base_to_map * odom_to_base.inverse();
    ros::Time ts = ros::Time::now();
    ros::Duration transform_tolerance(0.1);
    br_.sendTransform(tf::StampedTransform(odom_to_map, ts+transform_tolerance, "map", "odom"));

    odom_filter_pub_.publish(odom);
}


Eigen::VectorXd EKF::getState()
{
    return state_;
}

Eigen::MatrixXd EKF::getCovariance()
{
    return cov_state_;
}
