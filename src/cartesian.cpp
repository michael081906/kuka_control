#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>



int main(int argc, char * argv[]){
	ros::init(argc,argv, "cartesian");
	ros::NodeHandle nh_;
	int loop_freq = 10;
	ros::Rate loop_rate(loop_freq);
	geometry_msgs::PoseStamped cmd;
	ros::Publisher cmd_pub = nh_.advertise<geometry_msgs::PoseStamped>("iiwa/command/CartesianPose",10);

 	cmd.pose.position.x = 0.2;	
 	cmd.pose.position.y = 0.5;	
 	cmd.pose.position.z = 0.4;
	int ctr = 1;
	while(ros::ok()){
		cmd.header.stamp = ros::Time::now();
		cmd_pub.publish(cmd);
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
