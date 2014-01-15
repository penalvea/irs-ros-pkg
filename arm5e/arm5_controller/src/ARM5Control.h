#ifndef ARM5CONTROL_H
#define ARM5CONTROL_H

#define RPM_LIMIT	2000		//default RPM limit. Scale all the velocities accordingly
#define ARM5_CURRENT_LIMIT      0x0fff	//Low-level speed limit motor by motor
#define ARM5_SPEED_LIMIT        0xffff  	//Low-level current limit
#define ARM5_GRASPCURRENT_LIMIT      0x00ff	//Low-level current limit for the gripper
#define PGAIN		1		//default position gain
#define ARM5_SEC_CURRENT	2	//Max current threshold
#define maxT 1.5                        //
#define ARM5_VELDISCONT_LIMIT	0.4	//Velocity discontinuity limit. If diff. between the norm of one velocity and the previous is higher than this, send zero velocity
#define LIMIT_SECURITY_RANGE 0.05	//If less than LIMIT_SECURITY_RANGE to limit, ignore velocity towards limit direction

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "arm5_controller/setZero.h"
#include "arm5_controller/setPID.h"
//#include "arm5_controller/setParams.h"

#include "ARM5Coms.h"

//VISP
#include <visp/vpColVector.h>

#include <time.h>

double ratios[5]={216.0/29568.0, 8.0/1856.0, 8.0/1456.0, 0.0031415926535897933 /*8.0/1456.0*/, 0.00053702438522902445 /*8.0/384.0*/};
//double lmin[5]={250.0+17.0, 445.5+22.0, 390.0+47.0, 0, 0};
double lmin[5]={250.0+18.13, 445.5+22.0, 390.0+47.0, 0, 0};
double d[5]={322.85, 73.08, 489.02, 0, 0 };
double r[5]={58.16, 537.0, 53.86, 0, 0 };
//double aoffsets[5]={0.256456, 0.292535, 0.2475, 0, 0};
double aoffsets[5]={0.292535, 0.292535, 0.2475, 0, 0};
double moffsets[5]={-M_PI_2, 0, 2.15294, 0, 0};	///< Angle in radians according to the geom. model in init. configuration
double minlimits[5]={-M_PI_2, 0, 0, -M_PI, -M_PI};	///< Lower joint limit for each joint
//double ranges[5]={2.07383 /* For lmin_slew=20 */, 1.58665, 2.21479, 0, 0};
double ranges[5]={2.1195, 1.58665, 2.15294, 0, 0};


class ARM5Control
{
public:
  vpColVector axis_lmin, axis_ratios, axis_d, axis_r, axis_aoffsets, model_offsets, axis_ranges;

  vpColVector tick_offsets;	//offsets
  bool offsets_defined;

  vpColVector rawticks, rticks, ticks, axis_lengths, axis_alpha, q, qdot;	//positions
  double masterCurrent;

  vpColVector rpmlimit;
  int climit, slimit, gclimit; 		//low-level current and speed limit 0-4096
  bool securityStopEnabled;	//if true, it is checked that velocity reference arrives each 500ms. If not, set zero velocity
  bool emergencyStop;		//true if high current detected. If true, all velocities are set to zero.
  bool emergencyStopEnabled;

  double lastSignalTimer;	//Time when the last control signal was received

  double maxCurrentTime;
  bool maxCurrentEnabled;
  ARM5Control();

  void controlCycle();

  void setTicksVelocity(vpColVector &rpm);	///< Set joint velocity in RPM
  void setTicksVelocity(double rpm0, double rpm1, double rpm2, double rpm3, double rpm4);	///< Set joint velocity in RPM
  void setTicksVelocityLimit(vpColVector &limit) {rpmlimit=limit;}

  void setSpeedLimit(int slimit) {this->slimit=slimit;}
  void setCurrentLimit(int climit) {this->climit=climit;}
  void setGraspCurrentLimit(int gclimit) {this->gclimit=gclimit;}

  double getMasterCurrent() {return masterCurrent;}

  void enableEmergency(bool flag) {emergencyStopEnabled=flag;}
  ~ARM5Control() {if (coms!=NULL) delete coms;}

private:
  bool setZero(arm5_controller::setZero::Request  &req, arm5_controller::setZero::Response &res);
  void commandTicks(const sensor_msgs::JointState::ConstPtr& js);
  void commandlength(const sensor_msgs::JointState::ConstPtr& js);
  void commandAngle(const sensor_msgs::JointState::ConstPtr& js);
  bool setPID_callback(arm5_controller::setPID::Request  &req, arm5_controller::setPID::Response &res );
  //bool setParams_callback(arm5_controller::setParams::Request  &req, arm5_controller::setParams::Response &res);

  ARM5Coms *coms;		//Pointer to the communications object
  int coms_fd;			//File descriptor for communications

public:
  ros::NodeHandle nh_;
  ros::ServiceServer service;	//Service for setting the zero offsets
  ros::Publisher js_rticks_pub;	//Publish position/velocity/effort commands in relative tick units
  ros::Subscriber js_ticks_sub;	//Receive position/velocity/effort commands in abs tick units
  ros::Publisher js_ticks_pub;	//Publish position/velocity/effort commands in abs tick units
  ros::Publisher js_rawticks_pub;	//Publish position values in raw tick units as they are measured from the robot
  ros::Subscriber js_length_sub;	//Receive position/velocity/effort commands in axis length units (m & m/s)
  ros::Publisher js_length_pub;	//Publish position/velocity/effort commands in axis length units (m & m/s)
  ros::Subscriber js_angle_sub;	//Receive position/velocity/effort commands in angle units (rad & rad/s)
  ros::Publisher js_angle_pub;	//Publish position/velocity/effort commands in tick units (rad & rad/s)
  ros::ServiceServer PIDService; //Service for setting the PID params
  //ros::ServiceServer ParamService; //Service for setting the controller params
};

#endif


