#include<ros/ros.h>
#include<ros/package.h>
#include "Eigen/Dense"
#include<vector>
#include<fstream>
#include<iostream>
#include<geometry_msgs/Twist.h>

using namespace std;
void check_files(ifstream& in_file,string& in_name){
	if(!in_file.is_open()){
		cerr<< "Cannot open trajectory file"<< in_name<< endl;
		exit(EXIT_FAILURE);
	}	
}



int main(int argc, char * argv[]){

	Eigen::MatrixXd A(6,6);
	Eigen::VectorXd C(6);
	Eigen::VectorXd Pos(6);

	string trajectory_file_name;
	ros::init(argc,argv,"trajectory");
	ros::NodeHandle nh_;
	ros::NodeHandle home("~");

	int loop_frequency = 100;
	ros::Rate loop_rate(loop_frequency);	

	ros::Publisher dbg_pub = nh_.advertise<geometry_msgs::Twist>("/traj/dbg",10);
	geometry_msgs::Twist dbg;
	// getting the name of the trajecotry parameter file and reading it (it should be in a .txt file)
	home.getParam("trajectory_file_name", trajectory_file_name);
	trajectory_file_name = ros::package::getPath("kuka_control") + trajectory_file_name;
	ifstream trajectory_file(trajectory_file_name.c_str(), ifstream::in);
	check_files(trajectory_file,trajectory_file_name);
	string line;
	double ts;
	double tf;
	double xs;
	double xsdot;
	double xsddot;
	double xf;
	double xfdot;
	double xfddot;
	// for now only processes the last data set in the file as with the start and stop data
	while(getline(trajectory_file, line)){
		istringstream iss(line);
		iss>> ts;
		iss>> tf;
		iss>> xs;	
		iss>> xsdot;	
		iss>> xsddot;	
		iss>> xf;	
		iss>> xfdot;	
		iss>> xfddot;	
	}
	double ts2 =ts*ts; double ts3 = ts*ts2; double ts4=ts*ts3; double ts5=ts*ts4;
	double tf2 =tf*tf; double tf3 = tf*tf2; double tf4=tf*tf3; double tf5=tf*tf4;
	// the polynomial time-based matrix
	A << ts5,ts4,ts3,ts2,ts,1,
	     5*ts4,4*ts3,3*ts2,2*ts,1,0,
	     20*ts3,12*ts2,6*ts,2,0,0, 
	     tf5,tf4,tf3,tf2,tf,1,
	     5*tf4,4*tf3,3*tf2,2*tf,1,0,
	     20*tf3,12*tf2,6*tf,2,0,0;
	// boundary condition vector
	Pos << xs, xsdot,xsddot,xf, xfdot, xfddot;
	// find the coeeficient vector c5,c4,c3,c2,c1,c0
	C= A.inverse()*Pos;
	double t0 = ros::Time::now().toSec();
	double t = t0;
	// x is an example of the XYZ-RPY trajectories
	double x= 0;
	double xdot = 0;
	double xddot = 0;
	Eigen::VectorXd T(6);
	Eigen::VectorXd Tdot(6);
	Eigen::VectorXd Tddot(6);
	while(ros::ok()){
		t = ros::Time::now().toSec()-t0;
		if (t>= ts && t<= tf){
			double t2 = t*t; double t3= t*t2; double t4=t*t3; double t5=t*t4;
		 	T << t5,t4,t3,t2,t,1;	
		 	Tdot << 5*t4,4*t3,3*t2,2*t,1,0;	
		 	Tddot << 20*t3,12*t2,6*t,2,0,0; 
			x = T.transpose()*C;		
			xdot = Tdot.transpose()*C;
			xddot = Tddot.transpose()*C;
		}
		// variables for degugging and tracking the codes
		dbg.angular.x = t;
		dbg.linear.x = x;
		dbg.linear.y = xdot;
		dbg.linear.z = xddot;
		dbg_pub.publish(dbg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	trajectory_file.close();
	return 0;
}

