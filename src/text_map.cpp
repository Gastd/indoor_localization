#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "pose_interface");
    ros::NodeHandle n;

    tf::TransformBroadcaster broadcaster;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("ar_camera_pose", 0);
    geometry_msgs::PoseWithCovarianceStamped arPose_;

    // tf::Quaternion rotation (tf::Quaternion::getIdentity());
    tf::Quaternion rotation; rotation.setEuler(-M_PI, -0.0, -M_PI);
    tf::Vector3 origin(0.0, 0.0, 2.01);
    tf::Vector3 origin_odom(0.0, 0.0, 0.0);
    tf::Transform t(rotation, origin);

    tf::Transform markerToCam = t.inverse();
    tf::poseTFToMsg(t, arPose_.pose.pose);

    std::string frame_id = "4x4_1";
    arPose_.header.stamp = ros::Time::now();
    arPose_.header.frame_id = frame_id;
    double std_dev = 1 - 0.87; // y = 1 - x, x E [0,1]
    double var = std_dev*std_dev;
    // ROS_INFO("standart deviation equal to %lf", std_dev);
    arPose_.pose.covariance[0]  = var;
    arPose_.pose.covariance[7]  = var;
    arPose_.pose.covariance[14] = var;
    arPose_.pose.covariance[21] = var;
    arPose_.pose.covariance[28] = var;
    arPose_.pose.covariance[35] = var;

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        pub.publish(arPose_);
        broadcaster.sendTransform(tf::StampedTransform(t, ros::Time::now(), "usb_camera_optical_frame", frame_id));
        broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion::getIdentity(),
                                                     origin_odom), ros::Time::now(), "map", "odom"));
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
