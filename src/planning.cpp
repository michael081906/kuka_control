#include<ros/ros.h>
#include<ros/package.h>
#include<fstream>
#include<iostream>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 
#include<tf/transform_datatypes.h>

using namespace std;
void check_files(ifstream& in_file,string& in_name){
	if(!in_file.is_open()){
		cerr<< "Cannot open trajectory file"<< in_name<< endl;
		exit(EXIT_FAILURE);
	}	
}



int main(int argc, char * argv[]){

	string plan_file_name;
	ros::init(argc,argv,"planning");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

	int loop_frequency = 10;
	ros::Rate loop_rate(loop_frequency);	

	ros::Publisher plan_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("/plan",10);
	trajectory_msgs::JointTrajectory plan;
	trajectory_msgs::JointTrajectoryPoint point;
	for (int i = 0; i < 6; ++i){
		point.positions.push_back(0.0);
		point.velocities.push_back(0.0);
		point.accelerations.push_back(0.0);
	}
	// getting the name of the trajectory parameter file and reading it (it should be in a .txt file)
	home.getParam("plan_file_name", plan_file_name);
	plan_file_name = ros::package::getPath("kuka_control") + plan_file_name;
	ifstream plan_file(plan_file_name.c_str(), ifstream::in);
	check_files(plan_file,plan_file_name);
	string line;
	
	// for now only processes the last data set in the file as with the start and stop data
	int ctr = 0;
	while(getline(plan_file, line)){
		istringstream iss(line);
		if (ctr == 0){
			double dump;
			for (int i = 0; i < 18; ++i)
				iss >> dump;
			ctr ++;
		}else{
			// read the positions
			for (int i = 0; i < 6; ++i)
				iss >> point.positions[i];
			// read the velocities
			for (int i = 0; i < 6; ++i)
				iss >> point.velocities[i];
			// read the accelerations
			for (int i = 0; i < 6; ++i)
				iss >> point.accelerations[i];
			// for now we do not specify durations
			plan.points.push_back(point);
		}
	}
	
	while(ros::ok()){
		plan.header.stamp = ros::Time::now();
		plan_pub.publish(plan);
		ros::spinOnce();
		loop_rate.sleep();
	}
	plan_file.close();
	return 0;
}

