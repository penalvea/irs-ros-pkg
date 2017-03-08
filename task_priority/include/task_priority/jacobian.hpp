#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <Eigen/SVD>
#include <Eigen/Dense>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/shared_ptr.hpp>


class Jacobian
{
  Eigen::MatrixXd jac_;
  Eigen::MatrixXd jac_no_mask_;
public:
  Jacobian();
  virtual ~Jacobian();
  Eigen::MatrixXd getJac();
  Eigen::MatrixXd getJacNoMask();

  Eigen::MatrixXd pinvJac();
  Eigen::MatrixXd pinvJac(Eigen::MatrixXd jac);
  Eigen::MatrixXd kerJac();
  void setJac(Eigen::MatrixXd jac);
  void setJacNoMask(Eigen::MatrixXd jac);

};
typedef boost::shared_ptr<Jacobian> JacobianPtr;

class CartesianJacobian: public Jacobian{
  const KDL::Chain chain_;
  KDL::Chain chain_odom_;
  KDL::ChainJntToJacSolver *jntToJac_;
  std::vector<int> mask_cartesian_, mask_joint_;
  int n_joints_;
  std::vector<int> joints_relation_;
  std::vector<float> odom_;
  bool frame_inertial_;
public:
  CartesianJacobian(const KDL::Chain &chain, int n_joints, std::vector<int> joints_relation, std::vector<int> mask_cartesian, std::vector<int> mask_joint, bool frame_inertial);
  ~CartesianJacobian();
  bool calculateJac(std::vector<float> joints);
  void setMaskCartesian(std::vector<int> mask);
  void setMaskJoint(std::vector<int> mask);
  void setOdom(std::vector<float> odom);
};
typedef boost::shared_ptr<CartesianJacobian> CartesianJacobianPtr;


class JointJacobian: public Jacobian{
  int n_joints_;
  std::vector<int> mask_joint_;
  std::vector<int> joints_relation_;

public:
  JointJacobian(int n_joints, std::vector<int> joints_relation, std::vector<int> mask_joint);
  ~JointJacobian();
  bool calculateJac(std::vector<float> joints);
  void setMaskCartesian(std::vector<int> mask);
  void setMaskJoint(std::vector<int> mask);
  void setOdom(std::vector<float> odom);
};
typedef boost::shared_ptr<JointJacobian> JointJacobianPtr;



#endif // JACOBIAN_H
