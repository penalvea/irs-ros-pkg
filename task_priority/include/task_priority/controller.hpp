#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "task_priority/task.hpp"
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>


class Controller{

  std::vector<MultiTaskPtr> multitasks_;
  std::vector<float> max_joint_limit_, min_joint_limit_;
  int n_joints_;
  float acceleration_;
  float max_joint_vel_;
  float sampling_duration_;
  ros::NodeHandle nh_;
  ros::Publisher joints_pub_, vehicle_pub_;
  ros::Subscriber joints_sub_;
  tf::TransformListener listener;
  std::vector<float> current_joints_;
  bool joints_init_, goal_init_;
  std::string world_tf_, vehicle_tf_;

  void jointsCallback(const sensor_msgs::JointStateConstPtr  &msg);
  void publishVels(Eigen::MatrixXd vels);
  Eigen::MatrixXd limitVels(Eigen::MatrixXd vels);
  float calculateMaxNegativeVel(float current_joint, float min_joint_value, float acceleration, float sampling_duration);
  float calculateMaxPositiveVel(float current_joint, float max_joint_value, float acceleration, float sampling_duration);
public:
  Controller(std::vector<MultiTaskPtr> multitasks, int n_joints, std::vector<float> max_joint_limit, std::vector<float> min_joint_limit, float acceleration, float max_joint_vel, float sampling_duration, ros::NodeHandle nh, std::string arm_joint_state_topic, std::string arm_joint_command_topic, std::string vehicle_tf, std::string world_tf, std::string vehicle_command_topic);
  ~Controller();
  void goToGoal();

};
typedef boost::shared_ptr<Controller> ControllerPtr;

#endif // CONTROLLER_HPP