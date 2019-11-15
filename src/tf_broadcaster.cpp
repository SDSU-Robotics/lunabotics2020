   1 #include <ros/ros.h>
   2 #include <tf/transform_broadcaster.h>
   3 
   4 int main(int argc, char** argv){
   5   ros::init(argc, argv, "robot_tf_publisher");
   6   ros::NodeHandle n;
   7 
   8   ros::Rate r(100);
   9 
  10   tf::TransformBroadcaster broadcaster;
  11 
  12   while(n.ok()){
  13     broadcaster.sendTransform(
  14       tf::StampedTransform(
  15         tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
  16         ros::Time::now(),"base_link", "base_laser"));
  17     r.sleep();
  18   }
  19 }