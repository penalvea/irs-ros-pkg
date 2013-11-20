/* This is a node for initializing the CSIP ARM5E
   It moves the joints to the limits to set the absolute zero
*/
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


#include <ARM5Controller/setZero.h>

//TODO: calibrate
double manualZeroAbsTicks[5]={10841,0,0,0,0};

class InitCSIP
{
public:
  InitCSIP();

  bool initAuto;		//Whether to initialize automatically without user teleoperation
  bool initAxis[5];		//axis to initialize
  bool initAxisDone[5];		//axis already initialized
  int AxisDir[5];		//Direction to move (+1 or -1)
  double initAxisThreshold[5];    //Current threshold for each joint initialization
  int position[5];		//Current joint position
  int limitPosition[5];		//Joint position at the limit
  double current;		//Current the arm is consuming

  int activeAxis;

  ros::Time limitDetected;


  ros::NodeHandle nh_;
  ros::Publisher vel_pub_;	//Publish arm command velocity

  int callSetZeroService();
  void autoInitVel();

private:
  void stateCallback(const sensor_msgs::JointState::ConstPtr& state);
  //void joyCallback(const joy::Joy::ConstPtr& joy);

  int axisindex[6];		//joystick mapping
  double scale_;

  ros::Subscriber joy_sub_;	//Subscribe to joystick commands
  ros::Subscriber js_sub_;	//Subscribe to arm joint state
  ros::ServiceClient setZeroClient;
};

InitCSIP::InitCSIP()
{
  //Default values. Do not initialize any axis
  for (int i=0; i<5; i++) {
	initAxis[i]=false;
	initAxisDone[i]=false;
	AxisDir[i]=1;
	initAxisThreshold[i]=0.6;
	limitPosition[i]=0;
  }
  initAuto=false;

  //Read from the parameter server
  nh_.param("initAuto", initAuto, initAuto);
  if (initAuto) std::cerr << "Automatic initialization" << std::endl;
  nh_.param("initSlew", initAxis[0], initAxis[0]);
  nh_.param("initShoulder", initAxis[1], initAxis[1]);
  nh_.param("initElbow", initAxis[2], initAxis[2]);
  nh_.param("initWrist", initAxis[3], initAxis[3]);
  nh_.param("initJaw", initAxis[4], initAxis[4]);
  nh_.param("initSlewThreshold", initAxisThreshold[0], initAxisThreshold[0]);
  nh_.param("initShoulderThreshold", initAxisThreshold[1], initAxisThreshold[1]);
  nh_.param("initElbowThreshold", initAxisThreshold[2], initAxisThreshold[2]);
  nh_.param("initWristThreshold", initAxisThreshold[3], initAxisThreshold[3]);
  nh_.param("initJawThreshold", initAxisThreshold[4], initAxisThreshold[4]);
 
  for (int i=0; i<5; i++) {
	std::cerr << "Axis " << i << " threshold: " << initAxisThreshold[i] << std::endl;
  }

  //joystick mapping parameters
  if (!initAuto) { 
	  nh_.param("SlewAxis", axisindex[0], axisindex[0]);
	  nh_.param("ShoulderAxis", axisindex[1], axisindex[1]);
	  nh_.param("ElbowAxis", axisindex[2], axisindex[2]);
	  nh_.param("JawRotateAxis", axisindex[3], axisindex[3]);
	  nh_.param("JawCloseAxis", axisindex[4], axisindex[4]);
	  nh_.param("JawOpenAxis", axisindex[5], axisindex[5]);
  }

  scale_=2000;
  nh_.param("scale", scale_, scale_);
  nh_.param("SlewDir", AxisDir[0], AxisDir[0]);
  nh_.param("ShoulderDir", AxisDir[1], AxisDir[1]);
  nh_.param("ElbowDir", AxisDir[2], AxisDir[2]);
  nh_.param("WristDir", AxisDir[3], AxisDir[3]);
  nh_.param("JawDir", AxisDir[4], AxisDir[4]);
  

  activeAxis=0;
  //while (activeAxis<5 && !initAxis[activeAxis]) activeAxis++;
  //if (activeAxis<5)
  std::cerr << "Initializing joint " << activeAxis << ". Move the joint to the limit..." << std::endl;

  //publish joint velocity
  vel_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm5e/command_ticks",1);
  //subscribe to obtain joint feedback
  js_sub_ = nh_.subscribe<sensor_msgs::JointState>("/arm5e/joint_state_rticks", 1, &InitCSIP::stateCallback, this);
  //subscribe to joystick
  //if (!initAuto)
  	//joy_sub_ = nh_.subscribe<joy::Joy>("joy", 1, &InitCSIP::joyCallback, this);
  //service
  setZeroClient = nh_.serviceClient<ARM5Controller::setZero>("setZero");
}


void InitCSIP::stateCallback(const sensor_msgs::JointState::ConstPtr& state)
{
   //std::cerr << "Called stateCallack" << std::endl;
   //std::cerr << "state effort size: " << state->effort.size() <<  std::endl;
   
   if (state->effort.size()>0) {
	current=state->effort[0];
	for (int i=0; i<5; i++) position[i]=state->position[i];
   }

   if (activeAxis<5) { 
     if (initAxis[activeAxis]) {
		//std::cerr << "Current is: " << current << std::endl;
		//send velocity to activeAxis (done by joystick callback), check current
		if (current>initAxisThreshold[activeAxis] && !initAxisDone[activeAxis]) {
			//limit reached
			limitPosition[activeAxis]=position[activeAxis];
			initAxisDone[activeAxis]=true;
			std::cerr << "Joint " << activeAxis << " limit at position " << limitPosition[activeAxis] << std::endl;
			std::cerr << "Waiting 5 sec for the next" << std::endl;
			limitDetected = ros::Time::now();
		} else if (initAxisDone[activeAxis]) {
			//wait 5 sec.
			if ((ros::Time::now()-limitDetected).toSec() > 5) {
				//Move to the next joint
				//while (activeAxis<5 && (!initAxis[activeAxis] || initAxisDone[activeAxis])) activeAxis++;
				activeAxis++;
   				if (activeAxis<5) std::cerr << "Initializing joint " << activeAxis << ". Move the joint to the limit..." << std::endl;
			}
		}
     } else {
	//Do not initialize axis at the limit. Assume it has manually brought to zero and set offsets according to a previous calibration
	limitPosition[activeAxis]=position[activeAxis]-manualZeroAbsTicks[activeAxis];
	activeAxis++;
   	if (activeAxis<5) std::cerr << "Initializing joint " << activeAxis << ". Move the joint to the limit..." << std::endl;
     }
   }

}

/*void InitCSIP::joyCallback(const joy::Joy::ConstPtr& joy)
{
  //std::cerr << "Called joyCallback. Sending: " << std::endl;
  //std::cerr << (!initAxisDone[0] && initAxis[0])*scale_*joy->axes[axisindex[0]] << " " <<
//		(!initAxisDone[1] && initAxis[1])*scale_*joy->axes[axisindex[1]] << " " <<
//		(!initAxisDone[2] && initAxis[2])*scale_*joy->axes[axisindex[2]] << " " <<
//		(!initAxisDone[3] && initAxis[3])*scale_*joy->axes[axisindex[3]] << " " << std::endl;
  
  sensor_msgs::JointState js;
  js.name.push_back(std::string("Slew"));
  js.velocity.push_back((!initAxisDone[0] && initAxis[0])*scale_*joy->axes[axisindex[0]]*AxisDir[0]);
  js.name.push_back(std::string("Shoulder"));
  js.velocity.push_back((!initAxisDone[1] && initAxis[1])*scale_*joy->axes[axisindex[1]]*AxisDir[1]);
  js.name.push_back(std::string("Elbow"));
  js.velocity.push_back((!initAxisDone[2] && initAxis[2])*scale_*joy->axes[axisindex[2]]*AxisDir[2]);
  js.name.push_back(std::string("JawRotate"));
  js.velocity.push_back((!initAxisDone[3] && initAxis[3])*scale_*joy->axes[axisindex[3]]*AxisDir[3]);
  js.name.push_back(std::string("JawOpening"));
  if (joy->axes[axisindex[4]] < joy->axes[axisindex[5]]) {
  	js.velocity.push_back((!initAxisDone[4] && initAxis[4])*scale_*((joy->axes[axisindex[4]]-1)/2)*AxisDir[4]);
  } else if (joy->axes[axisindex[4]] > joy->axes[axisindex[5]]) {
  	js.velocity.push_back((!initAxisDone[4] && initAxis[4])*-scale_*((joy->axes[axisindex[5]]-1)/2)*AxisDir[4]);
  } else {
  	js.velocity.push_back(0);
  }
  vel_pub_.publish(js);
}

void InitCSIP::autoInitVel()
{ 
  sensor_msgs::JointState js;
  js.name.push_back(std::string("Slew"));
  js.name.push_back(std::string("Shoulder"));
  js.name.push_back(std::string("Elbow"));
  js.name.push_back(std::string("JawRotate"));
  js.name.push_back(std::string("JawOpening"));

  js.velocity.resize(5);
  for (int i=0; i<5; i++) js.velocity[i]=0;
  if (activeAxis<5 && initAxis[activeAxis] && !initAxisDone[activeAxis]) {
  	js.velocity[activeAxis]=AxisDir[4]*scale_;
  }
  vel_pub_.publish(js);
}*/


int InitCSIP::callSetZeroService() {
	ARM5Controller::setZero srv;

	for (int i=0; i<5; i++)
		srv.request.zeroOffsets.push_back(limitPosition[i]);

	if (setZeroClient.call(srv))
	{
		ROS_INFO("Zero offsets have been set: %d %d %d %d %d\n", limitPosition[0],limitPosition[1],limitPosition[2],limitPosition[3],limitPosition[4]);
	}
	else
	{
		ROS_ERROR("Failed to call service setZero");
		return 0;
	}
	return 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "csipInitialization");
  std::cerr << "Creating InitCSIP object" << std::endl;
  InitCSIP init_csip;

  bool offsetsDone=false;
 
  for (int i=0; i<10; i++) ros::spinOnce();
 
  while (ros::ok()) {
  	ros::spinOnce();
	//if (init_csip.initAuto)
		//init_csip.autoInitVel();		
	if (init_csip.activeAxis>=5 && !offsetsDone) {
	  std::cerr << "All axis initialized. Sending zero offsets to the arm..." << std::endl;
	  init_csip.callSetZeroService();	
	  offsetsDone=true;	
	  ros::shutdown();
	}
  }
  

  return 0;
}

