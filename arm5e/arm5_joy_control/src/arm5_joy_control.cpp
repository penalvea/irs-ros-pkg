#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#define CURRENT_THRESHOLD 6.0

class TeleopCSIP
{
public:
  TeleopCSIP();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void armCallback(const sensor_msgs::JointState::ConstPtr& mes);
  void armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes);
  
  ros::NodeHandle nh_;

  int axisindex[6];
  int AxisDir[5];
  int buttonindex[5], lastValue_[5];
  double error_;
  double scale_;
  double current;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber arm_sub_, arm_angle_sub_;
  bool slewLocked_, jawRotateLocked_, jawOpenLocked_, fixSlew_, park_;
  double angles_[5], desired_[5];
  
};


TeleopCSIP::TeleopCSIP()
{
  for (int i=0; i<5; i++)
        AxisDir[i]=1;
  current=0;
  slewLocked_=false;
  jawRotateLocked_=false;
  jawOpenLocked_=false;
  fixSlew_=false;
  park_=false; 
  nh_.param("SlewAxis", axisindex[0], axisindex[0]);
  nh_.param("ShoulderAxis", axisindex[1], axisindex[1]);
  nh_.param("ElbowAxis", axisindex[2], axisindex[2]);
  nh_.param("JawRotateAxis", axisindex[3], axisindex[3]);
  nh_.param("JawCloseAxis", axisindex[4], axisindex[4]);
  nh_.param("JawOpenAxis", axisindex[5], axisindex[5]);
  nh_.param("SlewDir", AxisDir[0], AxisDir[0]);
  nh_.param("ShoulderDir", AxisDir[1], AxisDir[1]);
  nh_.param("ElbowDir", AxisDir[2], AxisDir[2]);
  nh_.param("WristDir", AxisDir[3], AxisDir[3]);
  nh_.param("JawDir", AxisDir[4], AxisDir[4]);
  nh_.param("scale", scale_, scale_);   
  nh_.param("error", error_, error_);   
  nh_.param("LockSlewButton", buttonindex[0], buttonindex[0]);
  nh_.param("LockJawRotateButton", buttonindex[1], buttonindex[1]);  
  nh_.param("LockJawOpenButton", buttonindex[2], buttonindex[2]);
  nh_.param("FixSlewButton", buttonindex[3], buttonindex[3]);
  nh_.param("ParkButton", buttonindex[4], buttonindex[4]);
  
  vel_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm5e/command_ticks",1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("arm_joy", 10, &TeleopCSIP::joyCallback, this);
  arm_sub_= nh_.subscribe<sensor_msgs::JointState>("/arm5e/joint_state_rticks", 1, &TeleopCSIP::armCallback, this);
  arm_angle_sub_= nh_.subscribe<sensor_msgs::JointState>("/arm5e/joint_state_angle", 1, &TeleopCSIP::armAngleCallback, this);

  desired_[0]=0.0;
  desired_[1]=0.15;
  desired_[2]=1.8;
  desired_[3]=0.0;
  desired_[4]=0.36;

}


void TeleopCSIP::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	sensor_msgs::JointState js;

	if (joy->buttons[buttonindex[0]]==1 && lastValue_[0]!=1){
		slewLocked_=slewLocked_?false:true;
		ROS_INFO("Button %d pressed, Slew locked: %s", buttonindex[0], slewLocked_?"yes":"no" );
	}
	
    if (joy->buttons[buttonindex[1]]==1 && lastValue_[1]!=1){
		jawRotateLocked_=jawRotateLocked_?false:true;
		ROS_INFO("Button %d pressed, Jaw rotate locked: %s", buttonindex[1], jawRotateLocked_?"yes":"no" );
	}
	
	if (joy->buttons[buttonindex[2]]==1 && lastValue_[2]!=1){
		jawOpenLocked_=jawOpenLocked_?false:true;
		ROS_INFO("Button %d pressed, Jaw open locked: %s", buttonindex[2], jawOpenLocked_?"yes":"no" );
	}
	
	if (joy->buttons[buttonindex[3]]==1 && lastValue_[3]!=1){
		fixSlew_=fixSlew_?false:true;
		if(fixSlew_) park_=false;
		ROS_INFO("Button %d pressed, Fix slew angle: %s", buttonindex[3], fixSlew_?"yes":"no" );
	}
	
	if (joy->buttons[buttonindex[4]]==1 && lastValue_[4]!=1){
		park_=park_?false:true;
		if(park_)fixSlew_=false;
		ROS_INFO("Button %d pressed, Park: %s", buttonindex[4], park_?"yes":"no" );
	}
	

	if(current<CURRENT_THRESHOLD){
		if (park_){
			int okcount=0;int speed_=scale_; 
			js.name.push_back(std::string("Slew"));
			if( (angles_[0]-desired_[0]) > error_){
			    js.velocity.push_back(-speed_*(angles_[0]-desired_[0]));
			}else if ( (angles_[0]-desired_[0]) < -error_){
				js.velocity.push_back(-speed_*(angles_[0]-desired_[0]));
			}else{
				js.velocity.push_back(0);okcount++;
			}
			js.name.push_back(std::string("Shoulder"));
			if( (angles_[1]-desired_[1]) > error_){
			    js.velocity.push_back(-speed_*(angles_[1]-desired_[1]));
			}else if ( (angles_[1]-desired_[1]) < -error_){
				js.velocity.push_back(-speed_*(angles_[1]-desired_[1]));
			}else{
				js.velocity.push_back(0);okcount++;
			}
			js.name.push_back(std::string("Elbow"));
			if( (angles_[2]-desired_[2]) > error_){
			    js.velocity.push_back(speed_*(angles_[2]-desired_[2]));//This speed has the sign changed
			}else if ( (angles_[2]-desired_[2]) < -error_){
				js.velocity.push_back(speed_*(angles_[2]-desired_[2]));
			}else{
				js.velocity.push_back(0);okcount++;
			}
			js.name.push_back(std::string("JawRotate"));
			if( (angles_[3]-desired_[3]) > error_){
			    js.velocity.push_back(-speed_/4*(angles_[3]-desired_[3]));
			}else if ( (angles_[3]-desired_[3]) < -error_){
				js.velocity.push_back(-speed_/4*(angles_[3]-desired_[3]));
			}else{
				js.velocity.push_back(0);okcount++;
			}
			js.name.push_back(std::string("JawOpening"));
			if( (angles_[4]-desired_[4]) > error_){
			    js.velocity.push_back(-speed_*(angles_[4]-desired_[4]));
			}else if ( (angles_[4]-desired_[4]) < -error_){
				js.velocity.push_back(-speed_*(angles_[4]-desired_[4]));
			}else{
				js.velocity.push_back(0);okcount++;
			}			
			if(okcount==5){
				park_=false;
				ROS_INFO("Parked with a precision of %3f", error_);
			}
		}else if (fixSlew_){
			js.name.push_back(std::string("Slew"));
			if(angles_[0]>-error_ && angles_[0]<error_){//Near 0
				fixSlew_=false;
				ROS_INFO("Slew is at 0 angles (+-%f)", error_);
				js.velocity.push_back(0);
			}else{
				if(angles_[0]>0)
					js.velocity.push_back(-2250);
				else
					js.velocity.push_back(2250);
			}
			js.name.push_back(std::string("Shoulder"));
			js.velocity.push_back(0);
			js.name.push_back(std::string("Elbow"));
			js.velocity.push_back(0);
			js.name.push_back(std::string("JawRotate"));
			js.velocity.push_back(0);
			js.name.push_back(std::string("JawOpening"));
			js.velocity.push_back(0);
		}else{
			js.name.push_back(std::string("Slew"));
			if(joy->axes[axisindex[0]]>0.2 || joy->axes[axisindex[0]]<-0.2){
				js.velocity.push_back(scale_*joy->axes[axisindex[0]]*AxisDir[0]*(slewLocked_?0.0:1.0));
			}
			else{
				js.velocity.push_back(0);
			}
			js.name.push_back(std::string("Shoulder"));
			js.velocity.push_back(scale_*joy->axes[axisindex[1]]*AxisDir[1]);

			js.name.push_back(std::string("Elbow"));

			js.velocity.push_back(scale_*joy->axes[axisindex[2]]*AxisDir[2]);

			js.name.push_back(std::string("JawRotate"));

			js.velocity.push_back(0.3*scale_*joy->axes[axisindex[3]]*AxisDir[3]*(jawRotateLocked_?0.0:1.0));

			js.name.push_back(std::string("JawOpening"));
			if (joy->axes[axisindex[4]] < joy->axes[axisindex[5]]) {
				if(joy->axes[axisindex[4]]!=0)
					js.velocity.push_back(scale_*((joy->axes[axisindex[4]]-1)/2)*AxisDir[4]);
				else
					js.velocity.push_back(0);
			} else if (joy->axes[axisindex[4]] > joy->axes[axisindex[5]]) {
				if(joy->axes[axisindex[5]]!=0)
					js.velocity.push_back(-scale_*((joy->axes[axisindex[5]]-1)/2)*AxisDir[4]*(jawOpenLocked_?0.0:1.0));
				else
					js.velocity.push_back(0);
			} else {
				js.velocity.push_back(0);
			}
		}
	}
	else{
		js.name.push_back(std::string("Slew"));
		js.velocity.push_back(0);
		js.name.push_back(std::string("Shoulder"));
		js.velocity.push_back(0);
		js.name.push_back(std::string("Elbow"));
		js.velocity.push_back(0);
		js.name.push_back(std::string("JawRotate"));
		js.velocity.push_back(0);
		js.name.push_back(std::string("JawOpening"));
		js.velocity.push_back(0);
	}


	//FOR 8 DOF ARM: TO REMOVE. FIXME
	/*
  js.name.push_back(std::string("q6"));
  js.velocity.push_back(0);
  js.name.push_back(std::string("q7"));
  if (joy->axes[axisindex[4]] < joy->axes[axisindex[5]]) {
  	js.velocity.push_back(scale_*((joy->axes[axisindex[4]]-1)/2)*AxisDir[4]);
  } else if (joy->axes[axisindex[4]] > joy->axes[axisindex[5]]) {
  	js.velocity.push_back(-scale_*((joy->axes[axisindex[5]]-1)/2)*AxisDir[4]);
  } else {
  	js.velocity.push_back(0);
  }
  js.name.push_back(std::string("q8"));
  js.velocity.push_back(0);
	 */
	vel_pub_.publish(js);
	
	lastValue_[0]=joy->buttons[buttonindex[0]];
	lastValue_[1]=joy->buttons[buttonindex[1]];	
	lastValue_[2]=joy->buttons[buttonindex[2]];
	lastValue_[3]=joy->buttons[buttonindex[3]];
	lastValue_[4]=joy->buttons[buttonindex[4]];
}

void TeleopCSIP::armCallback(const sensor_msgs::JointState::ConstPtr& mes){
	current=mes->effort[0];
}

void TeleopCSIP::armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes){
	for(int i=0;i<5;i++) angles_[i]=mes->position[i];
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm5_joy_control");
	TeleopCSIP teleop_csip;

	ros::spin();
}

