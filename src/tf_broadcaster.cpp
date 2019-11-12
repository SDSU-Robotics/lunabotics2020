#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

std::string static_turtle_name;

int main(int argc, char **argv)
{
  ros::init(argc,argv, "tf_broadcaster");

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();

  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = "beacon_frame";
  
  static_transformStamped.transform.translation.x = 0;
  static_transformStamped.transform.translation.y = 0;
  static_transformStamped.transform.translation.z = 0;

  static_transformStamped.transform.rotation.x = 0;
  static_transformStamped.transform.rotation.y = 0;
  static_transformStamped.transform.rotation.z = 0;
  static_transformStamped.transform.rotation.w = 1;

  static_broadcaster.sendTransform(static_transformStamped);
  
  ros::spin();

  return 0;
};