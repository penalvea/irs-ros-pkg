#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class VehicleTeleop
{
public:
  VehicleTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;
public:
  int axisindex[4];
  int AxisDir[6];
  bool lt_pressed_once, rt_pressed_once;
  double lscale_, ascale_;
  ros::Publisher tw_pub_;
  ros::Subscriber joy_sub_;

};


VehicleTeleop::VehicleTeleop()
{
  for (int i=0; i<6; i++)
    AxisDir[i]=1;

  nh_.param("X", axisindex[0], axisindex[0]);
  nh_.param("Y", axisindex[1], axisindex[1]);
  nh_.param("Z", axisindex[2], axisindex[2]);
  nh_.param("Yaw", axisindex[3], axisindex[3]);
  //Roll and Pitch use RB,LB and RT,LT. Hardcoded control.
  nh_.param("XDir", AxisDir[0], AxisDir[0]);
  nh_.param("YDir", AxisDir[1], AxisDir[1]);
  nh_.param("ZDir", AxisDir[2], AxisDir[2]);
  nh_.param("YawDir", AxisDir[3], AxisDir[3]);
  nh_.param("PitchDir", AxisDir[4], AxisDir[4]);
  nh_.param("RollDir", AxisDir[5], AxisDir[5]);
  nh_.param("linear_scale", lscale_, lscale_);
  nh_.param("angular_scale", ascale_, ascale_);

  std::string topic_name("/twist_command");
  nh_.getParam("topic", topic_name);
  tw_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_name, 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/vehicle_joy", 1, &VehicleTeleop::joyCallback, this);

  lt_pressed_once = false;
  rt_pressed_once = false;
}


void VehicleTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;

  // X,Y,Z velocity commands
  twist.linear.x = joy->axes[axisindex[0]] * AxisDir[0] * lscale_;
  twist.linear.y = joy->axes[axisindex[1]] * AxisDir[1] * lscale_;
  twist.linear.z = joy->axes[axisindex[2]] * AxisDir[2] * lscale_;

  // Yaw?
  twist.angular.z = joy->axes[axisindex[3]] * AxisDir[3] * ascale_;

  if(joy->buttons[4]==1) twist.angular.x = ascale_ * 0.8 *  AxisDir[4]; //Avoid always using maximum value.
  if(joy->buttons[5]==1) twist.angular.x = ascale_ * 0.8 * -AxisDir[4];

  //Back triggers' value is 0 until they are pressed, then the default is 1.
  if( joy->axes[2] != 0 ) lt_pressed_once = true;
  if( joy->axes[5] != 0 ) rt_pressed_once = true;

  if( lt_pressed_once != 0 && joy->axes[2] != 1) twist.angular.y = (joy->axes[2] - 1) / - 2 * ascale_ *   AxisDir[5];
  if( rt_pressed_once != 0 && joy->axes[5] != 1) twist.angular.y = (joy->axes[5] - 1) / - 2 * ascale_ *  -AxisDir[5];

  tw_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicle_joy_control");
  VehicleTeleop vteleop;

  ros::spin();
}

