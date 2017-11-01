#include<ros/ros.h>
#include<trajectory_msgs/JointTrajectory.h> 
#include<trajectory_msgs/JointTrajectoryPoint.h> 
#include<geometry_msgs/Twist.h>
#include<iiwa_msgs/JointPosition.h>
#include<sensor_msgs/JointState.h>
#include<kdl/chain.hpp>
#include "Eigen/Core"
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <string>
#include <sstream>

//Frame KDL::Frame::DH_Craig1989 (double a, double alpha, double d, double theta)

KDL::Chain LWR(){

  KDL::Chain chain;

  //base
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),
        KDL::Frame::DH_Craig1989(0,0,0.34065,0)));

  //joint 1
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 2 
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0.4,0)));

  //joint 3
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0,M_PI_2,0,0)));

  //joint 4
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0.39,0)));

  //joint 5
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, -M_PI_2,0,0)));

  //joint 6
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        KDL::Frame::DH_Craig1989(0, M_PI_2,0,0)));

  //joint 7 (with flange adapter)
  chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
        //KDL::Frame::DH_Craig1989(0,0,0.098,0)));
    KDL::Frame::DH_Craig1989(0,0,0.088,0))); //AS

  return chain;

}

//reading the kuka lwr joint positions
sensor_msgs::JointState joints;
bool initialized = false;
//callback for reading joint values
void get_joints(const sensor_msgs::JointState & data){
	for (int i = 0; i < data.position.size();++i){
		// if this is not the first time the callback function is read, obtain the joint positions
		if(initialized){
			joints.position[i] = data.position[i];	
		// otherwise initilize them with 0.0 values
		}else{
			joints.position.push_back(0.0);
		}
	}	
	initialized = true;
}

// initialize the joint positions with a non-zero value to be used in the solvers
void initialize_joints(KDL::JntArray & _jointpositions, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_jointpositions(i) = _init;
}

// initialize a joint command point
void initialize_points(trajectory_msgs::JointTrajectoryPoint & _pt, int _nj, float _init){
	for (int i = 0; i < _nj; ++i)
		_pt.positions.push_back(_init);
}

//defines the joint names for the robot (used in the jointTrajectory messages)
void name_joints(trajectory_msgs::JointTrajectory & _cmd, int _nj){
	for (int i = 1; i <= _nj; ++i){
		std::ostringstream joint_name;		
		joint_name << "iiwa_joint_";
		joint_name << i;
		_cmd.joint_names.push_back(joint_name.str());
	}
}

// loads the joint space points to be sent as a command to the robot
void eval_points(trajectory_msgs::JointTrajectoryPoint & _point, KDL::JntArray & _jointpositions, int _nj){
	for (int i = 0; i < _nj; ++i)
		_point.positions[i] = _jointpositions(i);
		
}

// read the reference trajectory from the reflexxes node e.g. ref xyz-rpy
bool ref_received= false;
geometry_msgs::Twist ref;
void get_ref(const geometry_msgs::Twist & data){
	ref = data;
	ref_received = true;
}


int main(int argc, char * argv[]){
	// define the kinematic chain
	KDL::Chain chain = LWR();
	// define the forward kinematic solver via the defined chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
	// define the inverse kinematics solver
	KDL::ChainIkSolverVel_pinv iksolverv = KDL::ChainIkSolverVel_pinv(chain);//Inverse velocity solver
	KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	// get the number of joints from the chain
	unsigned int nj = chain.getNrOfJoints();
	// define a joint array in KDL format for the joint positions
    KDL::JntArray jointpositions = KDL::JntArray(nj);
	// define a joint array in KDL format for the next joint positions
	KDL::JntArray jointpositions_new = KDL::JntArray(nj);
	// define a manual joint command array for debugging	
	KDL::JntArray manual_joint_cmd = KDL::JntArray(nj);
	

	// define the ros node
	ros::init(argc,argv, "joint");
	ros::NodeHandle nh_;

	trajectory_msgs::JointTrajectory joint_cmd;
	trajectory_msgs::JointTrajectoryPoint pt,pt2;
    iiwa_msgs::JointPosition real_cmd;
	initialize_points(pt,nj,0.0);
	initialize_points(pt2,nj,0.0);
	
	ros::NodeHandle home("~");
	bool manual = true;
	bool reflexx = false;
	double roll, pitch, yaw, x, y, z;
	home.getParam("reflexx",reflexx);
	// in the manual mode, joint angle commands are directly sent to each joint of the robot
	home.getParam("manual",manual);
	for (int i = 0; i < nj; ++i){
		std::ostringstream joint_name;		
		joint_name << "j";
		joint_name << i+1;
		home.getParam(joint_name.str(),manual_joint_cmd(i));
	}
	// when not manually commanding the joints, xyz-rpy 
	// coordinates are read from the launch file to use 
	// inverse kinematics and command the joints to a desired location
	home.getParam("roll", roll);
	home.getParam("pitch",pitch);
	home.getParam("yaw",yaw);
	home.getParam("x",x);
	home.getParam("y",y);
	home.getParam("z",z);

	// setting up the loop frequency 
	int loop_freq = 10;
	float dt = (float) 1/loop_freq;
	ros::Rate loop_rate(loop_freq);

	
	// defining the puilsher that accepts joint position commands and applies them to the simulator
	ros::Publisher cmd_pub = nh_.advertise<trajectory_msgs::JointTrajectory>("iiwa/PositionJointInterface_trajectory_controller/command",10);
	// defining the puilsher that accepts joint position commands and applies them to the real kuka arm
	ros::Publisher real_cmd_pub = nh_.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition",10);

	// debugging publishers
	ros::Publisher dbg_pub = nh_.advertise<geometry_msgs::Twist>("mydbg",10);


	// subscriber for reading the joint angles from the gazebo simulator
	ros::Subscriber joints_sub = nh_.subscribe("/iiwa/joint_states",10, get_joints);
	// subscriber for reading the reference trajectories from the reflexxes-based node	
	ros::Subscriber ref_sub = nh_.subscribe("/reftraj",10, get_ref);

	
 
	// define the joint names, e.g. iiwa_joint_1 up to iiwa_joint_7
	name_joints(joint_cmd, nj);
	

	initialize_joints(jointpositions, nj, 0.2);

	KDL::Frame cartpos;    
	KDL::Rotation rpy = KDL::Rotation::RPY(roll,pitch,yaw); //Rotation built from Roll-Pitch-Yaw angles
 
	// for debugging: Calculate forward position kinematics
	bool kinematics_status;

	// apply the manual joint commands read from the launch file
	if (manual){
		eval_points(pt, manual_joint_cmd, nj);	
	}else{
		cartpos.p[0]=x;
		cartpos.p[1]=y;
		cartpos.p[2]=z;
		cartpos.M = rpy;
		int ret = iksolver.CartToJnt(jointpositions,cartpos,jointpositions_new);
		// get the target point ready after the inverse kinmatics is solved
		eval_points(pt, jointpositions_new, nj);
	}
	// debugging variables
	geometry_msgs::Twist xyz;

	pt.time_from_start = ros::Duration(1.0);
	pt2.time_from_start = ros::Duration(5.0);


	joint_cmd.points.push_back(pt);
	//cmd.points.push_back(pt2);
    bool real_robot = false;
	while(ros::ok()){		
		if (initialized){
			// update the joint positions with the most recent readings from the joints
			for (int k = 0; k<7; ++k){
				jointpositions(k) = joints.position[k];
			}				

			if(!manual && reflexx && ref_received){
				// update the reference cartesian positions
				cartpos.p[0]=ref.linear.x;
				cartpos.p[1]=ref.linear.y;
				cartpos.p[2]=ref.linear.z;			
				rpy = KDL::Rotation::RPY(ref.angular.x,ref.angular.y,ref.angular.z);
				cartpos.M = rpy;
				int ret = iksolver.CartToJnt(jointpositions,cartpos,jointpositions_new);
				eval_points(pt, jointpositions_new, nj);
				pt.time_from_start = ros::Duration(dt);
				joint_cmd.points[0] = pt;
			}

			kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
			if(kinematics_status>=0){
				xyz.linear.x = cartpos.p[0];
				xyz.linear.y = cartpos.p[1];
				xyz.linear.z = cartpos.p[2];
			}
			joint_cmd.header.stamp = ros::Time::now();
			cmd_pub.publish(joint_cmd);
			dbg_pub.publish(xyz);			
		}
		if(real_robot){
			real_cmd.header.stamp = ros::Time::now();
			real_cmd.position.a1 = -0.5;
			real_cmd.position.a2 = 0.9;
			real_cmd_pub.publish(real_cmd);
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
