#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_datatypes.h>

int main(int argc, char * argv[]) {
  ros::init(argc, argv, "cartesian");
  ros::NodeHandle nh_;
  int loop_freq = 10;
  ros::Rate loop_rate(loop_freq);
  geometry_msgs::PoseStamped cmd;
  ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::PoseStamped>(
      "iiwa/command/CartesianPose", 10);
  tf::Quaternion q = tf::createQuaternionFromRPY(3.14, 0, -3.14);
  q.normalize();

  cmd.pose.position.x = 0.4;
  cmd.pose.position.y = 0.0;
  cmd.pose.position.z = 0.6;
  cmd.pose.orientation.x=q.getX();
  cmd.pose.orientation.y=q.getY();
  cmd.pose.orientation.z=q.getZ();
  int ctr = 1;
  while (ros::ok()) {
    cmd.header.stamp = ros::Time::now();
    cmd_pub.publish(cmd);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
