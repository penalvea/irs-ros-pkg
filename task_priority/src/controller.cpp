#include <task_priority/controller.hpp>

Controller::Controller(std::vector<MultiTaskPtr> multitasks, int n_joints, std::vector<float> max_joint_limit, std::vector<float> min_joint_limit,float acceleration, float max_joint_vel, float sampling_duration, ros::NodeHandle nh, std::string arm_joint_state_topic, std::string arm_joint_command_topic, std::string vehicle_tf, std::string world_tf, std::string vehicle_command_topic){
multitasks_=multitasks;
n_joints_=n_joints;
max_joint_limit_=max_joint_limit;
min_joint_limit_=min_joint_limit;
acceleration_=acceleration;
max_joint_vel_=max_joint_vel;
sampling_duration_=sampling_duration;
world_tf_=world_tf;
vehicle_tf_=vehicle_tf;
nh_=nh;
joints_init_=false;
goal_init_=false;
current_joints_.resize(n_joints);


joints_sub_=nh_.subscribe<sensor_msgs::JointState>(arm_joint_state_topic, 1, &Controller::jointsCallback, this);
joints_pub_=nh_.advertise<sensor_msgs::JointState>(arm_joint_command_topic,1);
vehicle_pub_=nh_.advertise<nav_msgs::Odometry>(vehicle_command_topic, 1);

}

Controller::~Controller(){}

void Controller::jointsCallback(const sensor_msgs::JointStateConstPtr &msg){
  for(int i=0; i<4; i++){
    current_joints_[i]=0.0;
  }
  for(int i=4; i<8; i++){
    current_joints_[i]=msg->position[i-4];
  }
  joints_init_=true;
}

float Controller::calculateMaxPositiveVel(float current_joint, float max_joint_value, float acceleration, float sampling_duration){
  std::cout<<"calculate max positive vel"<<std::endl;
  std::cout<<max_joint_value<<"   "<<current_joint<<"   "<<sampling_duration<<std::endl;
  std::cout<<2*acceleration<<"    "<< max_joint_value-current_joint<<"    "<<2*acceleration*(max_joint_value-current_joint)<<std::endl;
  std::cout<<(max_joint_value-current_joint)/sampling_duration<<"    "<<std::sqrt(2*acceleration*(max_joint_value-current_joint))<<std::endl;
  return std::max(std::min((max_joint_value-current_joint)/sampling_duration, std::sqrt(2*acceleration*(max_joint_value-current_joint))),(float)0.0);
}

float Controller::calculateMaxNegativeVel(float current_joint, float min_joint_value, float acceleration, float sampling_duration){
  return std::min(std::max((min_joint_value-current_joint)/sampling_duration, -std::sqrt(2*acceleration*(current_joint-min_joint_value))),(float)0.0);
}

void Controller::goToGoal(){
  bool initialized=false;

  while(ros::ok() && !initialized){
    std::cout<<"Joints init "<<joints_init_<<std::endl;
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
      Eigen::MatrixXd T_k_complete;
      Eigen::MatrixXd vels(n_joints_,1);
      vels.setZero();
      for(int i=multitasks_.size()-1; i>=0; i--){
        std::cout<<"Entro multitask"<<std::endl;
        multitasks_[i]->setCurrentJoints(current_joints_);
        multitasks_[i]->setOdom(odom);
        multitasks_[i]->setMaxPositiveJointVelocity(max_positive_joint_velocities);
        multitasks_[i]->setMaxNegativeJointVelocity(max_negative_joint_velocities);
        vels=multitasks_[i]->calculateMultiTaskVel(vels, T_k_complete);
        std::cout<<vels<<std::endl;
        T_k_complete=multitasks_[i]->getT_k_complete();
      }
      vels=limitVels(vels);
      std::cout<<"vels to publish "<<std::endl;
      std::cout<<vels<<std::endl;
      std::cout<<"----------------------------"<<std::endl;
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



  sensor_msgs::JointState joint_msg;
  joint_msg.name.push_back("Slew");
  joint_msg.name.push_back("Shoulder");
  joint_msg.name.push_back("Elbow");
  joint_msg.name.push_back("JawRotate");
  joint_msg.name.push_back("JawOpening");

  for(int i=4; i<8; i++){
    joint_msg.velocity.push_back(vels(i,0));
  }
  joint_msg.velocity.push_back(0);
  joint_msg.header.stamp=ros::Time::now();

  vehicle_pub_.publish(odom_msg);
  joints_pub_.publish(joint_msg);
  ros::spinOnce();
}
