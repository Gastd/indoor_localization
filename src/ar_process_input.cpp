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

    tf::poseMsgToTF(camPose->pose.pose, markerToCam);
    // markerToCam = camToMarker.inverse();

    std::string id_str = camPose->header.frame_id;
    std::size_t found = id_str.find('_');
    int measurement_id = atoi(id_str.substr(found+1).c_str());

    std::vector<double> m = map_.get(measurement_id-1);
    tf::Transform mapToMarker;
    mapToMarker.setOrigin(tf::Vector3(m.at(0), m.at(1), 2.91));
    mapToMarker.setRotation(tf::createQuaternionFromRPY(-M_PI, 0., -M_PI/2));
    tf::Transform markerToMap = mapToMarker.inverse();

    tf::Transform mapToCam = mapToMarker * markerToCam;
    tf::Transform baseToMap = baseToCamStamped_ * mapToCam.inverse();
    tf::Transform mapToBase = baseToMap.inverse();
    ros::Time ts = ros::Time::now();
    ros::Duration transform_tolerance(0.2);
    tf::poseTFToMsg(mapToBase, robotPose_.pose.pose);
    // tf::StampedTransform mapToBaseTFStamp(baseToMap, ts+transform_tolerance, "base_link", "map");

    tf::Transform baseToMarker = baseToMap * mapToMarker;
    // ROS_INFO_STREAM("Marker   " << id_str << endl);
    // ROS_INFO_STREAM("Origin x " << baseToMarker.getOrigin().getX());
    // ROS_INFO_STREAM("Origin y " << baseToMarker.getOrigin().getY());
    // ROS_INFO_STREAM("Origin z " << baseToMarker.getOrigin().getZ());

    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(4,4),
    t = Eigen::MatrixXd::Zero(4,4),
    newCovariance = Eigen::MatrixXd::Zero(4,4);

    covariance(0,0) = camPose->pose.covariance[0];
    covariance(1,1) = camPose->pose.covariance[7];
    covariance(2,2) = camPose->pose.covariance[14];
    covariance(3,3) = 1;
    // covariance(4,4) = camPose->pose.covariance[28];
    // covariance(5,5) = camPose->pose.covariance[35];

    ROS_INFO_STREAM("--------------- covariance -----------" << endl << covariance);

    t << 
    mapToBase.getBasis()[0].getX(), mapToBase.getBasis()[1].getX(), mapToBase.getBasis()[2].getX(), mapToBase.getOrigin().getX(),
    mapToBase.getBasis()[3].getY(), mapToBase.getBasis()[4].getY(), mapToBase.getBasis()[5].getY(), mapToBase.getOrigin().getY(),
    mapToBase.getBasis()[6].getZ(), mapToBase.getBasis()[7].getZ(), mapToBase.getBasis()[8].getZ(), mapToBase.getOrigin().getZ(),
    0                             , 0                             , 0                             , 1;

    ROS_INFO_STREAM("--------------- t -----------" << endl << t);

    newCovariance = t * covariance * t.transpose();
    ROS_INFO_STREAM("--------------- newCovariance -----------" << endl << newCovariance);

    robotPose_.pose.covariance[0] = newCovariance(0,0);
    robotPose_.pose.covariance[7] = newCovariance(1,1);
    robotPose_.pose.covariance[14] = newCovariance(2,2);
    robotPose_.pose.covariance[21] = 1;
    // robotPose_.pose.covariance[28] = newCovariance(4,4);
    // robotPose_.pose.covariance[35] = newCovariance(5,5);

    robotPose_.header.frame_id = "map";
    robotPose_.header.stamp = ros::Time::now();

    pub_.publish(robotPose_);
}

ARPoseInterface::~ARPoseInterface() {}
