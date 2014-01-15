#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <cmath>        // std::abs

class Teleop
{
public:
  Teleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendSpeed();
  ros::NodeHandle nh_;
public:
  int axisindex[5];
  int AxisDir[5];
  bool moving;
  double scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ARM5Arm *robot;
  vpHomogeneousMatrix desired_bMe, bMe;
  vpColVector next_joints;
  double q3, q4;
  
};


Teleop::Teleop()
{
  for (int i=0; i<5; i++)
        AxisDir[i]=1;

   nh_.param("X", axisindex[0], axisindex[0]);	
   nh_.param("Y", axisindex[1], axisindex[1]);
   nh_.param("Z", axisindex[2], axisindex[2]);
   nh_.param("JawRotate", axisindex[3], axisindex[3]);
   nh_.param("JawOpenAxis", axisindex[4], axisindex[4]);
   nh_.param("JawCloseAxis", axisindex[5], axisindex[5]);
   nh_.param("XDir", AxisDir[0], AxisDir[0]);
   nh_.param("YDir", AxisDir[1], AxisDir[1]);
   nh_.param("ZDir", AxisDir[2], AxisDir[2]);
   nh_.param("JawRotateDir", AxisDir[3], AxisDir[3]);
   nh_.param("JawDir", AxisDir[4], AxisDir[4]);
   nh_.param("scale", scale_, scale_);
	
 moving=false;

  vel_pub_ = nh_.advertise<sensor_msgs::JointState>("/uwsim/joint_state_comma",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &Teleop::joyCallback, this);
  robot=new ARM5Arm(nh_, "uwsim/joint_state", "uwsim/joint_state_command");
}


void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if(!moving && (std::abs(joy->axes[axisindex[0]])>0.05||std::abs(joy->axes[axisindex[1]])>0.05||std::abs(joy->axes[axisindex[2]])>0.05)){

			desired_bMe=bMe;
			desired_bMe[0][3]-=0.3*joy->axes[axisindex[0]]*AxisDir[0];
			desired_bMe[1][3]-=0.3*joy->axes[axisindex[1]]*AxisDir[1];
			desired_bMe[2][3]-=0.3*joy->axes[axisindex[2]]*AxisDir[2];

			//std::cerr<<"Desired bMe"<<std::endl<<desired_bMe<<std::endl;
			next_joints=robot->armIK(desired_bMe);
			//std::cerr<<"Desired joints"<<std::endl<<next_joints<<std::endl;
			
			//If valid joints and reasonable new position ... ask to MOVE
			if(next_joints[0]>-1.57 && next_joints[0]<2.1195 && next_joints[1]>0 && next_joints[1]<1.58665 && next_joints[2]>0 && next_joints[2]<2.15294){
				if(std::abs(desired_bMe[0][3]-bMe[0][3])<5 && std::abs(desired_bMe[1][3]-bMe[1][3])<5 && std::abs(bMe[2][3]-bMe[2][3])<5)
					moving=true;
				else
					ROS_INFO("Error: New position too far form the original position.");
			}else
				ROS_INFO("Error: Unreachable position.");
				
	}
	//Save Jaw movements for later use.
	q3=joy->axes[axisindex[3]];
        q4=joy->axes[axisindex[4]];
	//TODO Send speed: to avoid depending on joystick callbacks could be called from 
	//a while loop on main program setting desired rate. Current version doesn't sendSpeed if
	//if /joy topic is silent.
	sendSpeed();
}



void Teleop::sendSpeed()
{
	sensor_msgs::JointState js;
	vpColVector current_joints(5),send_joints(5);
	int res=robot->getJointValues(current_joints);
	if(res){		
		bMe=robot->directKinematics(current_joints);
		//std::cerr<<"Current bMe"<<std::endl<<bMe<<std::endl;
		
		if(moving){
			//Check if it's almost there
			if(std::abs(desired_bMe[0][3]-bMe[0][3])>0.01 || std::abs(desired_bMe[1][3]-bMe[1][3])>0.01|| std::abs(desired_bMe[2][3]-bMe[2][3])>0.01)
			{
				ROS_INFO("Info: Movingo to desired position.");
				send_joints[0]=next_joints[0]-current_joints[0];
				send_joints[1]=next_joints[1]-current_joints[1];
				send_joints[2]=next_joints[2]-current_joints[2];
			}
			else{
				ROS_INFO("Info: Position reached");
				send_joints[0]=0;
				send_joints[1]=0;
				send_joints[2]=0;
				moving=false;
			}
		}else{
			
				//ROS_INFO("Info: Joystick is not moving.");
				send_joints[0]=0;
				send_joints[1]=0;
				send_joints[2]=0;		
		}
	}
	else{
		ROS_ERROR("Error: Impossible to access to the arm controler");
		send_joints[0]=0;
		send_joints[1]=0;
		send_joints[2]=0;
	}
  //Jaw movement. Should be modified to meet requirements.
  send_joints[3]=scale_*q3*AxisDir[3];
  if (q3 < q4) {
  	send_joints[4]=scale_*((q3-1)/2)*AxisDir[4];
  } else if (q3 > q4) {
  	send_joints[4]=-scale_*((q4-1)/2)*AxisDir[4];
  } else {
  	send_joints[4]=0;
  }
  robot->setJointVelocity(send_joints);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm5_joy_control");
  Teleop teleop;

  ros::spin();
}

