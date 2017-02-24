#include <task_priority/goal.hpp>


Goal::Goal(){
  initialized_=false;
}
Goal::~Goal(){}

Eigen::Vector3d Goal::quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current){
  Eigen::Vector3d v1(quat_desired.x(), quat_desired.y(), quat_desired.z());
  Eigen::Vector3d v1_aux(quat_desired.x(), quat_desired.y(), quat_desired.z());
  Eigen::Vector3d v2(quat_current.x(), quat_current.y(), quat_current.z());
  Eigen::Vector3d v2_aux(quat_current.x(), quat_current.y(), quat_current.z());
  v1_aux.cross(v2);
  double norm=v1_aux.norm();
  double angle=std::atan2(norm, v1.adjoint()*v2);
  if(angle>(M_PI/2)+0.000000000001){
    quat_desired.x()=-quat_desired.x();
    quat_desired.y()=-quat_desired.y();
    quat_desired.z()=-quat_desired.z();
    quat_desired.w()=-quat_desired.w();
  }

  return quat_current.w()*v1-quat_desired.w()*v2+v2_aux.cross(v1);
}
void Goal::setInitialized(bool initialized){
  initialized_=initialized;
}
bool Goal::getInitialized(){
  return initialized_;
}




GoalFixedPose::GoalFixedPose(Eigen::MatrixXd goal, KDL::Chain chain, std::vector<int> mask_cart, std::vector<int> joints_relation):Goal(){
  KDL::Chain chain_odom;
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  chain_odom.addChain(chain);

  fk_chain_=new KDL::ChainFkSolverPos_recursive(chain_odom);
  mask_cart_=mask_cart;
  goal_=goal;
  joints_relation_=joints_relation;
  setInitialized(true);
}

GoalFixedPose::~GoalFixedPose(){}


Eigen::MatrixXd GoalFixedPose::getGoal(std::vector<float> joints, std::vector<float> odom){
  KDL::JntArray jq(odom.size()+joints_relation_.size());
  for(int i=0; i<odom.size(); i++){
    jq(i)=odom[i];
  }
  for(int i=0; i<joints_relation_.size(); i++){
    jq(i+odom.size())=joints[joints_relation_[i]];
  }

  KDL::Frame cartpos;
  fk_chain_->JntToCart(jq, cartpos);
  double x,y,z,w;
  cartpos.M.GetQuaternion(x,y,z,w);
  Eigen::Quaterniond quat_current(w,x,y,z);
  Eigen::Matrix3d current;
  current=Eigen::AngleAxisd(goal_(5,0), Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(goal_(4,0), Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(goal_(3,0), Eigen::Vector3d::UnitX());
  Eigen::Quaterniond quat_desired(current);
  Eigen::Vector3d diff=quaternionsSubstraction(quat_desired, quat_current);
  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0.0)=goal_(0,0)-cartpos.p.x();
  cartesian_vel(1.0)=goal_(1,0)-cartpos.p.y();
  cartesian_vel(2.0)=goal_(2,0)-cartpos.p.z();
  cartesian_vel(3.0)=diff[0];
  cartesian_vel(4.0)=diff[1];
  cartesian_vel(5.0)=diff[2];
  for(int i=0; i<cartesian_vel.rows(); i++){
    if(mask_cart_[i]==0){
      cartesian_vel(i,0)=0;
    }
  }
  return cartesian_vel;
}


GoalROSPose::GoalROSPose(KDL::Chain chain, std::vector<int> mask_cart, std::string pose_topic, ros::NodeHandle &nh, std::vector<int> joints_relation):Goal(){
  KDL::Chain chain_odom;
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  chain_odom.addChain(chain);

  fk_chain_=new KDL::ChainFkSolverPos_recursive(chain_odom);
  mask_cart_=mask_cart;
  joints_relation_=joints_relation;
  nh_=nh;

  pose_sub_=nh_.subscribe<geometry_msgs::Pose>(pose_topic, 1, &GoalROSPose::poseCallback, this);
}
GoalROSPose::~GoalROSPose(){}

void GoalROSPose::poseCallback(const geometry_msgs::Pose::ConstPtr &msg){
  std::cout<<"recivo pose"<<std::endl;
  goal_.orientation=msg->orientation;
  goal_.position=msg->position;
  setInitialized(true);
}

Eigen::MatrixXd GoalROSPose::getGoal(std::vector<float> joints, std::vector<float> odom){
  KDL::JntArray jq(odom.size()+joints_relation_.size());
  for(int i=0; i<odom.size(); i++){
    jq(i)=odom[i];
  }
  for(int i=0; i<joints_relation_.size(); i++){
    jq(i+odom.size())=joints[joints_relation_[i]];
  }

  KDL::Frame cartpos;
  fk_chain_->JntToCart(jq, cartpos);
  double x,y,z,w;
  cartpos.M.GetQuaternion(x,y,z,w);
  Eigen::Quaterniond quat_current(w,x,y,z);
  Eigen::Quaterniond quat_desired(goal_.orientation.w, goal_.orientation.x, goal_.orientation.y, goal_.orientation.z);
  Eigen::Vector3d diff=quaternionsSubstraction(quat_desired, quat_current);
  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0.0)=goal_.position.x-cartpos.p.x();
  cartesian_vel(1.0)=goal_.position.y-cartpos.p.y();
  cartesian_vel(2.0)=goal_.position.z-cartpos.p.z();
  cartesian_vel(3.0)=diff[0];
  cartesian_vel(4.0)=diff[1];
  cartesian_vel(5.0)=diff[2];
  for(int i=0; i<cartesian_vel.rows(); i++){
    if(mask_cart_[i]==0){
      cartesian_vel(i,0)=0;
    }
  }
  return cartesian_vel;
}

GoalROSTwist::GoalROSTwist(std::vector<int> mask_cart, std::string twist_topic, ros::NodeHandle &nh):Goal(){
  mask_cart_=mask_cart;
  nh_=nh;

  twist_sub_=nh_.subscribe<geometry_msgs::Twist>(twist_topic, 1, &GoalROSTwist::twistCallback, this);
}
GoalROSTwist::~GoalROSTwist(){}

void GoalROSTwist::twistCallback(const geometry_msgs::Twist::ConstPtr &msg){
  goal_.angular=msg->angular;
  goal_.linear=msg->linear;
  setInitialized(true);
}

Eigen::MatrixXd GoalROSTwist::getGoal(std::vector<float> joints, std::vector<float> odom){
  Eigen::MatrixXd cartesian_vel(6,1);
  cartesian_vel(0,0)=goal_.linear.x;
  cartesian_vel(1,0)=goal_.linear.y;
  cartesian_vel(2,0)=goal_.linear.z;
  cartesian_vel(3,0)=goal_.angular.x;
  cartesian_vel(4,0)=goal_.angular.y;
  cartesian_vel(5,0)=goal_.angular.z;
  return cartesian_vel;
}