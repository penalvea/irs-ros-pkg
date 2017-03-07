#include <task_priority/jacobian.hpp>
#include <iostream>


Jacobian::Jacobian(){}
Jacobian::~Jacobian(){}

Eigen::MatrixXd Jacobian::pinvJac(){
  return pinvJac(jac_);
}
Eigen::MatrixXd Jacobian::pinvJac(Eigen::MatrixXd jac){
  double epsilon=std::numeric_limits<double>::epsilon();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance=epsilon*std::max(jac.cols(), jac.rows())*svd.singularValues().array().abs()(0);
  return svd.matrixV()*(svd.singularValues().array().abs()>tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()*svd.matrixU().adjoint();
}

Eigen::MatrixXd Jacobian::getJac(){
  return jac_;
}
Eigen::MatrixXd Jacobian::getJacNoMask(){
  return jac_no_mask_;
}


Eigen::MatrixXd Jacobian::kerJac(){
  Eigen::MatrixXd ident=Eigen::MatrixXd::Identity(jac_.cols(), jac_.cols());
  return ident-(pinvJac()*jac_);
}


void Jacobian::setJac(Eigen::MatrixXd jac){
  jac_=jac;
}
void Jacobian::setJacNoMask(Eigen::MatrixXd jac){
  jac_no_mask_=jac;
}








CartesianJacobian::CartesianJacobian(const KDL::Chain &chain, int n_joints, std::vector<int> joints_relation, std::vector<int> mask_cartesian, std::vector<int> mask_joint, bool frame_inertial)
  :Jacobian(), chain_(chain){

  frame_inertial_=frame_inertial;
  KDL::Chain auv;
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  auv.addChain(chain);
  chain_odom_.addChain(auv);
  jntToJac_=new KDL::ChainJntToJacSolver(chain_odom_);
  mask_cartesian_=mask_cartesian;
  mask_joint_=mask_joint;
  n_joints_=n_joints;
  joints_relation_=joints_relation;

}

CartesianJacobian::~CartesianJacobian(){}

bool CartesianJacobian::calculateJac(std::vector<float> joints){
  KDL::JntArray q(chain_odom_.getNrOfJoints());
  if(frame_inertial_){
    for(int i=0; i<odom_.size(); i++){
      q(i)=odom_[i];
    }
  }
  else{
    for(int i=0; i<odom_.size(); i++){
      q(i)=0.0;
    }
  }
  for(int i=0; i<chain_.getNrOfJoints(); i++){
    q(i+odom_.size())=joints[joints_relation_[i]];
  }

  KDL::Jacobian jac(chain_odom_.getNrOfJoints());
  jntToJac_->JntToJac(q, jac);

  Eigen::MatrixXd jacobian(jac.rows(), n_joints_);
  jacobian.setZero();
  for(int j=0; j<joints_relation_.size(); j++){
    for(int i=0; i<jac.rows(); i++){
      jacobian(i,joints_relation_[j])=jac(i,j+odom_.size());
    }
  }
  Eigen::MatrixXd jacobian_no_mask=jacobian;


  if(mask_cartesian_.size()==jacobian.rows()){
    for(int i=0; i<mask_cartesian_.size(); i++){
      if(mask_cartesian_[i]==0){
        for(int j=0; j<jacobian.cols(); j++){
          jacobian(i,j)=0;
        }
      }
    }
  }
  if(mask_joint_.size()==jacobian.cols()){
    for(int j=0; j<mask_joint_.size(); j++){
      if(mask_joint_[j]==0){
        for(int i=0; i<jacobian.rows(); i++){
          jacobian(i,j)=0;
        }
      }
    }
  }
  Eigen::MatrixXd jacobian_complete(6, n_joints_);
  Eigen::MatrixXd jacobian_complete_no_mask(6, n_joints_);
  jacobian_complete.setZero();
  jacobian_complete_no_mask.setZero();
  for(int j=0; j<joints_relation_.size(); j++){
    for(int i=0; i<jac.rows(); i++){
      jacobian_complete(i, joints_relation_[j])=jacobian(i,j);
      jacobian_complete_no_mask(i, joints_relation_[j])=jacobian_no_mask(i,j);
    }
  }

  setJac(jacobian_complete);
  setJacNoMask(jacobian_complete_no_mask);
  return  true;


}
void CartesianJacobian::setMaskCartesian(std::vector<int> mask){
  mask_cartesian_=mask;
}
void CartesianJacobian::setMaskJoint(std::vector<int> mask){
  mask_joint_=mask;
}
void CartesianJacobian::setOdom(std::vector<float> odom){
  odom_=odom;
}


JointJacobian::JointJacobian(int n_joints, std::vector<int> joints_relation, std::vector<int> mask_joint)
  :Jacobian(){

  mask_joint_=mask_joint;
  n_joints_=n_joints;
  joints_relation_=joints_relation;

}

JointJacobian::~JointJacobian(){}

bool JointJacobian::calculateJac(std::vector<float> joints){

  Eigen::MatrixXd jacobian(n_joints_, n_joints_);
  jacobian=Eigen::MatrixXd::Identity(n_joints_, n_joints_);
  Eigen::MatrixXd jacobian_no_mask=jacobian;
  if(mask_joint_.size()==jacobian.cols()){
    for(int j=0; j<mask_joint_.size(); j++){
      if(mask_joint_[j]==0){
        for(int i=0; i<jacobian.rows(); i++){
          jacobian(i,j)=0;
        }
      }
    }
  }

  setJac(jacobian);
  setJacNoMask(jacobian_no_mask);
  return  true;


}
void JointJacobian::setMaskCartesian(std::vector<int> mask){
}
void JointJacobian::setMaskJoint(std::vector<int> mask){
  mask_joint_=mask;
}
void JointJacobian::setOdom(std::vector<float> odom){
}



