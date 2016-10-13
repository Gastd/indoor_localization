#include "indoor_localization/ar_process_input.h"
#include <cstdlib>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "pose_interface");
  ros::NodeHandle n;
  ARPoseInterface ar_interface(n);
  ros::spin ();
  return 0;
}

ARPoseInterface::ARPoseInterface(ros::NodeHandle& n): n_(n), mode2D(false)
{
    arPoseTopic_ = "ar_camera_pose";
    sub_ = n_.subscribe(arPoseTopic_, 1, &ARPoseInterface::arPoseCallback, this);

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
    pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ar_robot_pose", 0);
}

void ARPoseInterface::arPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& camPose)
{
    // Transform camera's pose in landmark frame to robot's pose in map frame and publish
    tf::Transform camToMarker, markerToCam;
    camToMarker.setOrigin(tf::Vector3(camPose->pose.pose.position.x,
                                      camPose->pose.pose.position.y,
                                      camPose->pose.pose.position.z));
    camToMarker.setRotation(tf::Quaternion(camPose->pose.pose.orientation.x,
                                           camPose->pose.pose.orientation.y,
                                           camPose->pose.pose.orientation.z,
                                           camPose->pose.pose.orientation.w));
    tf::Quaternion rotation_camera;
    markerToCam = camToMarker.inverse();
    rotation_camera.setRPY(0., 0., 0.); // rotation of camera optical frame
    rotation_camera = markerToCam*rotation_camera;
    markerToCam.setRotation(rotation_camera.normalized());
    double yaw = tf::getYaw(markerToCam.getRotation());
    camToMarker = markerToCam.inverse();

    std::string id_str = camPose->header.frame_id;
    std::size_t found = id_str.find('_');
    int measurement_id = atoi(id_str.substr(found+1).c_str());

    std::vector<double> m = map_.get(measurement_id-1);
    tf::Transform mapToMarker;
    tf::Quaternion rotationTag;
    mapToMarker.setOrigin(tf::Vector3(m.at(0), m.at(1), 2.91));
    mapToMarker.setRotation(tf::Quaternion(0., 0., 0., 1.));
    tf::Transform markerToMap = mapToMarker.inverse();
    rotationTag.setRPY(0., 0., M_PI/2);
    rotationTag = markerToMap*rotationTag;
    markerToMap.setRotation(rotationTag.normalized());
    mapToMarker = markerToMap.inverse();
    rotationTag.setRPY(M_PI, 0., 0.);
    rotationTag = mapToMarker*rotationTag;
    mapToMarker.setRotation(rotationTag.normalized());

    tf::Transform mapToCam = mapToMarker * camToMarker;
    tf::Transform mapToBase = baseToCamStamped_ * mapToCam.inverse();
    tf::Transform baseToMap = mapToBase.inverse();
    ros::Time ts = ros::Time::now();
    ros::Duration transform_tolerance(0.2);
    tf::poseTFToMsg(baseToMap, robotPose_.pose.pose);
    // tf::StampedTransform mapToBaseTFStamp(mapToBase, ts+transform_tolerance, "base_link", "map");

    robotPose_.header.frame_id = "map";
    robotPose_.header.stamp = ros::Time::now();

    pub_.publish(robotPose_);
}

ARPoseInterface::~ARPoseInterface() {}
