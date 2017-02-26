#ifndef TASK_HPP
#define TASK_HPP
#include <vector>
#include "task_priority/jacobian.hpp"
#include "task_priority/task_velocity.hpp"
#include <boost/shared_ptr.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include "goal.hpp"


class Task{
  CartesianJacobianPtr jac_;
  CartesianTaskVelocityPtr task_velocity_;
  std::vector<int> mask_cartesian_;
  std::vector<int> mask_joint_;
  KDL::Chain chain_;
  std::vector<int> chain_joint_relation_;
  bool active_;
  GoalPtr goal_;



public:
  Task(const KDL::Chain& chain, std::vector<int> mask_cartesian, int n_joints, std::vector<int> mask_joint, std::vector<int> chain_joint_relation, GoalPtr goal, bool frame_inertial);
  ~Task();
  void activate();
  void deactivate();
  bool isActive();
  Eigen::MatrixXd calculateCartesianError(Eigen::MatrixXd current_joint_vel, std::vector<float> joints, std::vector<float> odom);
  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd getJacobianNoMask();
  void calculateJacobian(std::vector<float> current_joints, std::vector<float> odom);
  bool goalInitialized();
};
typedef boost::shared_ptr<Task> TaskPtr;



class MultiTask{
  std::vector<float> current_joints_;
  std::vector<TaskPtr> tasks_;
  std::vector<float> max_positive_joint_vel_, max_negative_joint_vel_, min_cartesian_limits_, max_cartesian_limits_;
  Eigen::MatrixXd T_k_complete_;
  Eigen::MatrixXd T_k_;
  Eigen::MatrixXd J_k_;
  Eigen::MatrixXd J_null_inv_;
  Eigen::MatrixXd J_k_no_mask_;
  std::vector<float> odom_;
  float max_vel_;
  std::vector<KDL::Chain> chains_;
  std::vector<std::vector<int> > chain_joint_relations_;
  std::vector<std::vector<float> > max_positive_cartesian_vel_, max_negative_cartesian_vel_;
  std::vector<std::vector<int> > joints_priority_;
  Eigen::MatrixXd pinvMat(Eigen::MatrixXd matrix);

public:
  MultiTask( std::vector<TaskPtr> tasks, std::vector<KDL::Chain> chains, std::vector<std::vector<int> > chain_joint_relations, std::vector<std::vector<int> > joints_priority);
  ~MultiTask();
  Eigen::MatrixXd getJacobian();
  std::vector<float> calculateCartesianVelocity();
  void calculateJacobians(Eigen::MatrixXd T_k_complete);
  void adaptJacobiansTask(std::vector<int> joints_active);


  void activate(int task);
  void deactivate(int task);
  void setMaxPositiveJointVelocity(std::vector<float> vels);
  void setMaxNegativeJointVelocity(std::vector<float> vels);
  void setMaxPositiveCartesianVelocity(std::vector<std::vector<float> > vels);
  void setMaxNegativeCartesianVelocity(std::vector<std::vector<float> > vels);
  bool isActive();
  void setCurrentJoints(std::vector<float> joints);
  Eigen::MatrixXd calculateJointsVel(Eigen::MatrixXd error, std::vector<int> joints_active);
  Eigen::MatrixXd calculateError(Eigen::MatrixXd last_vel);
  Eigen::MatrixXd calculateMultiTaskVel(Eigen::MatrixXd last_vel, Eigen::MatrixXd T_k_complete);
  void setOdom(std::vector<float> odom);
  Eigen::MatrixXd getT_k_complete();
  Eigen::MatrixXd limitJointsAndCartesian(Eigen::MatrixXd vels);
  bool goalsInitialized();
};

typedef boost::shared_ptr<MultiTask> MultiTaskPtr;




#endif // TASK_HPP
