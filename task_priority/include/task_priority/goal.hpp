#ifndef GOAL_HPP
#define GOAL_HPP
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <task_priority/Error_msg.h>


class Goal{
  bool initialized_;
  float max_cartesian_vel_;
public:
  Goal();
  virtual ~Goal();
  virtual Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom)=0;
  virtual task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask)=0;
  Eigen::Vector3d quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current);
  void setInitialized(bool initialized);
  bool getInitialized();
  void setMaxCartesianVel(float max_cartesian_vel);
  Eigen::MatrixXd limitCaresianVel(Eigen::MatrixXd vels);
};
typedef boost::shared_ptr<Goal> GoalPtr;


class GoalFixedPose: public Goal{
  KDL::ChainFkSolverPos_recursive *fk_chain_;
  std::vector<int> mask_cart_;
  Eigen::MatrixXd goal_;
  std::vector<int> joints_relation_;
  Eigen::MatrixXd last_cartesian_vel_;
public:
  GoalFixedPose(Eigen::MatrixXd goal, KDL::Chain chain, std::vector<int> mask_cart, std::vector<int> joints_relation, float max_cartesian_vel);
  ~GoalFixedPose();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalFixedPose> GoalFixedPosePtr;

class GoalROSPose: public Goal{

  KDL::ChainFkSolverPos_recursive *fk_chain_;
  std::vector<int> mask_cart_;
  geometry_msgs::Pose goal_;
  std::vector<int> joints_relation_;
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
   Eigen::MatrixXd last_cartesian_vel_;
  void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);
public:
  GoalROSPose(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation, float max_cartesian_vel);
  ~GoalROSPose();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalROSPose> GoalROSPosePtr;


class GoalROSTwist: public Goal{

  std::vector<int> mask_cart_;
  geometry_msgs::Twist goal_;
  ros::NodeHandle nh_;
  ros::Subscriber twist_sub_;
  Eigen::MatrixXd last_cartesian_vel_;
  void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);
public:
  GoalROSTwist(std::vector<int> mask_cart, std::string twist_topic, ros::NodeHandle &nh, float max_cartesian_vel);
  ~GoalROSTwist();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalROSTwist> GoalROSTwistPtr;

class GoalJointsPosition: public Goal{
  std::vector<float> joints_position_;
   Eigen::MatrixXd last_joint_vel_;
public:
  GoalJointsPosition(std::vector<float> joints_position);
  ~GoalJointsPosition();
  Eigen::MatrixXd getGoal(std::vector<float> joints, std::vector<float> odom);
  task_priority::Error_msg getMsg(Eigen::MatrixXd vels, std::vector<int> mask);
};
typedef boost::shared_ptr<GoalJointsPosition> GoalJointsPositionPtr;

#endif // GOAL_HPP
