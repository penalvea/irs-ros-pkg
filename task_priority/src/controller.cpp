#include <task_priority/controller.hpp>

Controller::Controller(std::vector<MultiTaskPtr> multitasks, int n_joints, std::vector<float> max_joint_limit, std::vector<float> min_joint_limit, std::vector<std::vector<float> > max_cartesian_limits, std::vector<std::vector<float> > min_cartesian_limits, float acceleration, float max_joint_vel, float sampling_duration, ros::NodeHandle nh, std::string arm_joint_state_topic, std::string arm_joint_command_topic, std::string vehicle_tf, std::string world_tf, std::string vehicle_command_topic, std::vector<KDL::Chain> chains, std::vector<std::vector<int> > chain_joint_relations){
multitasks_=multitasks;
n_joints_=n_joints;
max_joint_limit_=max_joint_limit;
min_joint_limit_=min_joint_limit;
max_cartesian_limits_=max_cartesian_limits;
min_cartesian_limits_=min_cartesian_limits;
acceleration_=acceleration;
max_joint_vel_=max_joint_vel;
sampling_duration_=sampling_duration;
world_tf_=world_tf;
vehicle_tf_=vehicle_tf;
nh_=nh;
joints_init_=false;
goal_init_=false;
current_joints_.resize(n_joints);
chains_=chains;
chain_joint_relations_=chain_joint_relations;




joints_sub_=nh_.subscribe<sensor_msgs::JointState>(arm_joint_state_topic, 1, &Controller::jointsCallback, this);
joints_pub_=nh_.advertise<sensor_msgs::JointState>(arm_joint_command_topic,1);
//vehicle_pub_=nh_.advertise<nav_msgs::Odometry>(vehicle_command_topic, 1);
vehicle_pub_=nh_.advertise<auv_msgs::BodyVelocityReq>(vehicle_command_topic, 1);

}

Controller::~Controller(){}

void Controller::jointsCallback(const sensor_msgs::JointStateConstPtr &msg){
  for(int i=0; i<5; i++){
    current_joints_[i]=0.0;
  }

  for(int i=5; i<9; i++){
    current_joints_[i]=msg->position[i-5];
  }
  joints_init_=true;
}

float Controller::calculateMaxPositiveVel(float current_joint, float max_joint_value, float acceleration, float sampling_duration){
  if(current_joint>max_joint_value)
    return 0.0;
  return std::max(std::min((max_joint_value-current_joint)/sampling_duration, std::sqrt(2*acceleration*(max_joint_value-current_joint))),(float)0.0);
}

float Controller::calculateMaxNegativeVel(float current_joint, float min_joint_value, float acceleration, float sampling_duration){
  if(current_joint<min_joint_value)
    return 0.0;
  return std::min(std::max((min_joint_value-current_joint)/sampling_duration, -std::sqrt(2*acceleration*(current_joint-min_joint_value))),(float)0.0);
}

float Controller::calculateMaxPositiveVel(float difference, float acceleration, float sampling_duration){
  if(difference<0)
    return 0.0;
  return std::max(std::min(difference/sampling_duration, std::sqrt(2*acceleration*difference)),(float)0.0);
}

float Controller::calculateMaxNegativeVel(float difference, float acceleration, float sampling_duration){
  if(difference>0)
    return 0.0;
  return std::min(std::max(difference/sampling_duration, -std::sqrt(2*acceleration*(-difference))),(float)0.0);
}

Eigen::Vector3d Controller::quaternionsSubstraction(Eigen::Quaterniond quat_desired, Eigen::Quaterniond quat_current){
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

void Controller::goToGoal(){
  bool initialized=false;

  while(ros::ok() && !initialized){
    if(joints_init_){
      initialized=true;
      for(int i=0; i<multitasks_.size(); i++){
        if(!multitasks_[i]->goalsInitialized()){
          initialized=false;
        }
      }
    }
    ros::Duration(sampling_duration_).sleep();
    ros::spinOnce();
  }
  std::cout<<"Topics initialized"<<std::endl;
  while(ros::ok()){
    try{
      tf::StampedTransform transform;
      listener.lookupTransform(world_tf_, vehicle_tf_, ros::Time(0), transform);
      Eigen::Quaterniond odom_rotation(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
      Eigen::Vector3d odom_euler=odom_rotation.toRotationMatrix().eulerAngles(0,1,2);
      std::vector<float> odom(6,0);
      odom[0]=transform.getOrigin().x();
      odom[1]=transform.getOrigin().y();
      odom[2]=transform.getOrigin().z();
      odom[3]=odom_euler[0];
      odom[4]=odom_euler[1];
      odom[5]=odom_euler[2];



      std::vector<float> max_positive_joint_velocities, max_negative_joint_velocities;
      for(int i=0; i<n_joints_; i++){
        max_positive_joint_velocities.push_back(calculateMaxPositiveVel(current_joints_[i], max_joint_limit_[i], acceleration_, sampling_duration_));
        max_negative_joint_velocities.push_back(calculateMaxNegativeVel(current_joints_[i], min_joint_limit_[i], acceleration_, sampling_duration_));
      }
      std::vector<std::vector<float> > max_negative_cartesian_velocities, max_positive_cartesian_velocities;
      std::vector<std::vector<std::vector<float> > > max_cartesian_vels=calculateMaxCartesianVels(current_joints_, odom);
      max_positive_cartesian_velocities=max_cartesian_vels[0];
      max_negative_cartesian_velocities=max_cartesian_vels[1];
      Eigen::MatrixXd T_k_complete;
      Eigen::MatrixXd vels(n_joints_,1);
      vels.setZero();
      for(int i=multitasks_.size()-1; i>=0; i--){
        multitasks_[i]->setCurrentJoints(current_joints_);
        multitasks_[i]->setOdom(odom);
        multitasks_[i]->setMaxPositiveJointVelocity(max_positive_joint_velocities);
        multitasks_[i]->setMaxNegativeJointVelocity(max_negative_joint_velocities);
        multitasks_[i]->setMaxPositiveCartesianVelocity(max_positive_cartesian_velocities);
        multitasks_[i]->setMaxNegativeCartesianVelocity(max_negative_cartesian_velocities);
        vels=multitasks_[i]->calculateMultiTaskVel(vels, T_k_complete);
        T_k_complete=multitasks_[i]->getT_k_complete();
      }
      vels=limitVels(vels);
      //std::cout<<"vels to publish "<<std::endl;
      //std::cout<<vels<<std::endl;
      //std::cout<<"----------------------------"<<std::endl;
      publishVels(vels);

    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s\n", ex.what());
      ros::Duration(sampling_duration_).sleep();
    }
    ros::Duration(sampling_duration_).sleep();
  }
}


Eigen::MatrixXd Controller::limitVels(Eigen::MatrixXd vels){
  float max_vel=0;
  for(int i=0; i<vels.rows(); i++){
    if(std::abs(vels(i,0))>max_vel){
      max_vel=std::abs(vels(i,0));
    }
  }
  if(max_vel>max_joint_vel_){
    for(int i=0; i<vels.rows(); i++){
      vels(i,0)=vels(i,0)*max_joint_vel_/max_vel;
    }
  }
  return vels;
}

void Controller::publishVels(Eigen::MatrixXd vels){
  nav_msgs::Odometry odom_msg;

  odom_msg.pose.pose.position.x=0.0;
  odom_msg.pose.pose.position.y=0.0;
  odom_msg.pose.pose.position.z=0.0;
  odom_msg.pose.pose.orientation.x=0.0;
  odom_msg.pose.pose.orientation.y=0.0;
  odom_msg.pose.pose.orientation.z=0.0;
  odom_msg.pose.pose.orientation.w=1.0;

  odom_msg.twist.twist.linear.x=vels(0,0);
  odom_msg.twist.twist.linear.y=vels(1,0);
  odom_msg.twist.twist.linear.z=vels(2,0);
  odom_msg.twist.twist.angular.x=0;
  odom_msg.twist.twist.angular.y=0;
  odom_msg.twist.twist.angular.z=vels(3,0);
  for (int i=0; i<36; i++) {
        odom_msg.twist.covariance[i]=0;
        odom_msg.pose.covariance[i]=0;
      }
  odom_msg.header.stamp=ros::Time::now();

  auv_msgs::BodyVelocityReq vehicle_msg;

  vehicle_msg.header.stamp=ros::Time::now();
  vehicle_msg.header.frame_id="girona500";
  vehicle_msg.goal.requester="irslab";
  vehicle_msg.goal.id=0;
  vehicle_msg.goal.priority=10;
  vehicle_msg.twist.linear.x=0.5*vels(0,0);
  vehicle_msg.twist.linear.y=0.5*vels(1,0);
  vehicle_msg.twist.linear.z=0.3*vels(2,0);
  vehicle_msg.twist.angular.x=0;
  vehicle_msg.twist.angular.y=0;
  vehicle_msg.twist.angular.z=0.5*vels(3,0);

  vehicle_msg.disable_axis.roll=true;
  vehicle_msg.disable_axis.pitch=true;





  sensor_msgs::JointState joint_msg;
  joint_msg.name.push_back("Slew");
  joint_msg.name.push_back("Shoulder");
  joint_msg.name.push_back("Elbow");
  joint_msg.name.push_back("JawRotate");
  joint_msg.name.push_back("JawOpening");

  for(int i=5; i<9; i++){
    joint_msg.velocity.push_back(5*vels(i,0));
  }
  joint_msg.velocity.push_back(0);
  joint_msg.header.stamp=ros::Time::now();

  //vehicle_pub_.publish(odom_msg);
  vehicle_pub_.publish(vehicle_msg);
  joints_pub_.publish(joint_msg);
  ros::spinOnce();
}


std::vector<std::vector<std::vector<float> > > Controller::calculateMaxCartesianVels(std::vector<float> joints, std::vector<float> odom){
  std::vector<std::vector<float> > max_negative_cartesian_vels, max_positive_cartesian_vels;
  for(int i=0; i<chains_.size(); i++){
    KDL::Chain chain_odom;
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    chain_odom.addChain(chains_[i]);
    KDL::ChainFkSolverPos_recursive fk(chain_odom);

    KDL::JntArray q(chain_odom.getNrOfJoints());
    for(int j=0; j<odom.size(); j++){
      q(j)=odom[j];

    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom.size())=joints[chain_joint_relations_[i][j]];
    }
    //for(int j=0; j<q.rows(); j++){
     // std::cout<<q(j)<<"  ";
    //}
    //std::cout<<std::endl;

    KDL::Frame frame;
    fk.JntToCart(q, frame);
    std::vector<float> max_negative_cartesian_vel, max_positive_cartesian_vel;
    //std::cout<<"Direct kinematics"<<std::endl;

    //std::cout<<frame.p.x()<<" "<<frame.p.y()<<" "<<frame.p.z()<<std::endl;
    double x_a, y_a, z_a, w_a;
    frame.M.GetQuaternion(x_a, y_a, z_a, w_a);
    //std::cout<<x_a<<" "<<y_a<<" "<<z_a<<" "<<w_a<<std::endl;

    max_negative_cartesian_vel.push_back(calculateMaxNegativeVel(frame.p.x(), min_cartesian_limits_[i][0], acceleration_, sampling_duration_));
    max_negative_cartesian_vel.push_back(calculateMaxNegativeVel(frame.p.y(), min_cartesian_limits_[i][1], acceleration_, sampling_duration_));
    max_negative_cartesian_vel.push_back(calculateMaxNegativeVel(frame.p.z(), min_cartesian_limits_[i][2], acceleration_, sampling_duration_));


    double q_x, q_y, q_z, q_w;
    frame.M.GetQuaternion(q_x, q_y, q_z, q_w);
    Eigen::Quaterniond current_rotation(q_w, q_x, q_y, q_z);
    Eigen::Matrix3d min_cartesian_mat;
    double min_x, min_y, min_z;
    if(min_cartesian_limits_[i][3]<-6.28){
      min_x=0;
    }
    else{
      min_x=min_cartesian_limits_[i][3];
    }
    if(min_cartesian_limits_[i][4]<-6.28){
      min_y=0;
    }
    else{
      min_y=min_cartesian_limits_[i][4];
    }
    if(min_cartesian_limits_[i][5]<-6.28){
      min_z=0;
    }
    else{
      min_z=min_cartesian_limits_[i][5];
    }
    min_cartesian_mat=Eigen::AngleAxisd(min_z, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(min_y, Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(min_z, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quat_limit_min(min_cartesian_mat);
    /*std::cout<<"quaternion min"<<std::endl;
    std::cout<<quat_limit_min.x()<<" "<<quat_limit_min.y()<<" "<<quat_limit_min.z()<<" "<<quat_limit_min.w()<<std::endl;
    std::cout<<"quaternion current"<<std::endl;
    std::cout<<current_rotation.x()<<" "<<current_rotation.y()<<" "<<current_rotation.z()<<" "<<current_rotation.w()<<std::endl;*/
    Eigen::Vector3d rot_dif=quaternionsSubstraction(quat_limit_min, current_rotation);
    //std::cout<<"diff"<<std::endl;
    //std::cout<<rot_dif<<std::endl;

    //std::cout<<"current_rotation"<<rot_dif<<std::endl;

    if(min_cartesian_limits_[i][3]<-6.28){
      max_negative_cartesian_vel.push_back(-1);
    }
    else{
      max_negative_cartesian_vel.push_back(calculateMaxNegativeVel(rot_dif[0], acceleration_, sampling_duration_));
    }
    if(min_cartesian_limits_[i][4]<-6.28){
      max_negative_cartesian_vel.push_back(-1);
    }
    else{
      max_negative_cartesian_vel.push_back(calculateMaxNegativeVel(rot_dif[1], acceleration_, sampling_duration_));
    }
    if(min_cartesian_limits_[i][5]<-6.28){
     max_negative_cartesian_vel.push_back(-1);
    }
    else{
      max_negative_cartesian_vel.push_back(calculateMaxNegativeVel(rot_dif[2], acceleration_, sampling_duration_));
    }
    max_negative_cartesian_vels.push_back(max_negative_cartesian_vel);

    max_positive_cartesian_vel.push_back(calculateMaxPositiveVel(frame.p.x(), max_cartesian_limits_[i][0], acceleration_, sampling_duration_));
    max_positive_cartesian_vel.push_back(calculateMaxPositiveVel(frame.p.y(), max_cartesian_limits_[i][1], acceleration_, sampling_duration_));
    max_positive_cartesian_vel.push_back(calculateMaxPositiveVel(frame.p.z(), max_cartesian_limits_[i][2], acceleration_, sampling_duration_));

    Eigen::Matrix3d max_cartesian_mat;
    double max_x, max_y, max_z;
    if(max_cartesian_limits_[i][3]>6.28){
      max_x=0;
    }
    else{
      max_x=max_cartesian_limits_[i][3];
    }
    if(max_cartesian_limits_[i][4]>6.28){
      max_y=0;
    }
    else{
      max_y=max_cartesian_limits_[i][4];
    }
    if(max_cartesian_limits_[i][5]>6.28){
      max_z=0;
    }
    else{
      max_z=max_cartesian_limits_[i][5];
    }
    max_cartesian_mat=Eigen::AngleAxisd(max_z, Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(max_y, Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(max_z, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quat_limit_max(max_cartesian_mat);
    /*std::cout<<"quaternion max"<<std::endl;
    std::cout<<quat_limit_max.x()<<" "<<quat_limit_max.y()<<" "<<quat_limit_max.z()<<" "<<quat_limit_max.w()<<std::endl;
    std::cout<<"quaternion current"<<std::endl;
    std::cout<<current_rotation.x()<<" "<<current_rotation.y()<<" "<<current_rotation.z()<<" "<<current_rotation.w()<<std::endl;*/
    rot_dif=quaternionsSubstraction(quat_limit_max, current_rotation);
   // std::cout<<"diff"<<std::endl;
    //std::cout<<rot_dif<<std::endl;

    //std::cout<<"current_rotation"<<rot_dif<<std::endl;

    if(max_cartesian_limits_[i][3]>6.28){
      max_positive_cartesian_vel.push_back(1);
    }
    else{
      max_positive_cartesian_vel.push_back(calculateMaxPositiveVel(rot_dif[0], acceleration_, sampling_duration_));
    }
    if(max_cartesian_limits_[i][4]>6.28){
      max_positive_cartesian_vel.push_back(1);
    }
    else{
      max_positive_cartesian_vel.push_back(calculateMaxPositiveVel(rot_dif[1], acceleration_, sampling_duration_));
    }
    if(max_cartesian_limits_[i][5]>6.28){
     max_positive_cartesian_vel.push_back(1);
    }
    else{
      max_positive_cartesian_vel.push_back(calculateMaxPositiveVel(rot_dif[2], acceleration_, sampling_duration_));
    }
    max_positive_cartesian_vels.push_back(max_positive_cartesian_vel);


  }
  std::vector<std::vector<std::vector<float> > > max_vels;
  max_vels.push_back(max_positive_cartesian_vels);
  max_vels.push_back(max_negative_cartesian_vels);
  return max_vels;
}

