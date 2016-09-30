#ifndef AR_POSE_AR_MULTI_H
#define AR_POSE_AR_MULTI_H


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// #include <image_transport/image_transport.h>

#include <indoor_localization/ar_map.h>


class ARPoseInterface
{
public:
    ARPoseInterface(ros::NodeHandle& n);
    ~ARPoseInterface();

private:
    ros::NodeHandle n_;
    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener listener_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    bool mode2D;

    std::string arPoseTopic_;

    tf::StampedTransform baseToCamStamped_;

    ARMap map_;
    geometry_msgs::PoseWithCovarianceStamped robotPose_;
    
    void arPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
};


#endif
