
#include <ros/ros.h>
#include <my_path_service/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<my_path_service::PathSrv>("path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    my_path_service::PathSrv path_srv;
    
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    
    pose.position.x = 0.0;
    pose.position.y = 3.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    
    pose.position.y = -6.5;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    pose.position.y = 8.0;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    pose.position.y = 4.0;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    pose.position.y = -1.5;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    pose.position.y = 2.0;
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    client.call(path_srv);
    return 0;
}
