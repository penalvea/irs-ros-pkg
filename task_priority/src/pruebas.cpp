#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <tf_conversions/tf_kdl.h>
#include <task_priority/task.hpp>

std::vector<float> joints_arm(4,0.0);
bool joints_init=false;

void jointsCallback(const sensor_msgs::JointStateConstPtr& msg){
  for(int i=0; i<4; i++){
    joints_arm[i]=msg->position[i];
  }
  joints_init=true;
}


Eigen::Vector3d restaQuaternion(Eigen::Quaterniond quat_d, Eigen::Quaterniond quat){
  Eigen::Vector3d v1(quat_d.x(), quat_d.y(), quat_d.z());
  Eigen::Vector3d v1_aux(quat_d.x(), quat_d.y(), quat_d.z());
  Eigen::Vector3d v2(quat.x(), quat.y(), quat.z());
  Eigen::Vector3d v2_aux(quat.x(), quat.y(), quat.z());
  v1_aux.cross(v2);
  double norm=v1_aux.norm();
  double angle=std::atan2(norm, v1.adjoint()*v2);
  if(angle>(M_PI/2)+0.000000000001){
    quat_d.x()=-quat_d.x();
    quat_d.y()=-quat_d.y();
    quat_d.z()=-quat_d.z();
    quat_d.w()=-quat_d.w();
  }



  return quat.w()*v1-quat_d.w()*v2+v2_aux.cross(v1);
}

Eigen::MatrixXd calculateGoalVel(Eigen::MatrixXd goal, KDL::Chain chain, std::vector<float> joints, std::vector<float> odom, float max_vel, std::vector<int> mask_cart){
  KDL::Chain chain_odom;
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
  chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
  chain_odom.addChain(chain);

  KDL::ChainFkSolverPos_recursive fk(chain_odom);

  KDL::JntArray jq(odom.size()+joints.size());
  for(int i=0; i<odom.size(); i++){
    jq(i)=odom[i];
  }
  for(int i=0; i<joints.size(); i++){
    jq(i+odom.size())=joints[i];
  }
  KDL::Frame cartpos;
  fk.JntToCart(jq, cartpos);
  tf::Pose pose;
  tf::poseKDLToTF(cartpos, pose);

  double roll, pitch, yaw;

  tf::Matrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);

  Eigen::MatrixXd goal_vel(6,1);

  goal_vel(0,0)=goal(0,0)-pose.getOrigin().x();
  goal_vel(1,0)=goal(1,0)-pose.getOrigin().y();
  goal_vel(2,0)=goal(2,0)-pose.getOrigin().z();

  for(int i=3; i<6; i++){
    if(goal(i,0)>M_PI)
      goal(i,0)=-(2*M_PI)+goal(i,0);
    else if(goal(i,0)<-M_PI)
      goal(i,0)=(2*M_PI)+goal(i,0);
  }
  if(roll>M_PI)
    roll=-(2*M_PI)+roll;
  else if(roll<-M_PI)
    roll=(2*M_PI)+roll;
  if(pitch>M_PI)
    pitch=-(2*M_PI)+pitch;
  else if(pitch<-M_PI)
    pitch=(2*M_PI)+pitch;
  if(yaw>M_PI)
    yaw=-(2*M_PI)+yaw;
  else if(roll<-M_PI)
    yaw=(2*M_PI)+yaw;

  Eigen::Matrix3d desired;

  desired=Eigen::AngleAxisd(goal(5,0), Eigen::Vector3d::UnitZ())*
      Eigen::AngleAxisd(goal(4,0), Eigen::Vector3d::UnitY())*
      Eigen::AngleAxisd(goal(3,0), Eigen::Vector3d::UnitX());
 Eigen::Matrix3d current;
 current=Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*
     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*
     Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  Eigen::Quaterniond desired_q(desired);
  Eigen::Quaterniond current_q(current);


  Eigen::Vector3d euler_dist=restaQuaternion(desired_q, current_q);

  goal_vel(3,0)=euler_dist[0];
  goal_vel(4,0)=euler_dist[1];
  goal_vel(5,0)=euler_dist[2];



  /*goal_vel(3,0)=goal(3,0)-roll;
  goal_vel(4,0)=goal(4,0)-pitch;
  goal_vel(5,0)=goal(5,0)-yaw;*/





  /*float max=max_vel;
  for(int i=0; i<3; i++){
    if(mask_cart[i]==1){
      if(std::abs(goal_vel(i,0))>max)
        max=std::abs(goal_vel(i,0));
    }
  }
  if(max>max_vel){
    for(int i=0; i<3; i++){
      goal_vel(i,0)=max_vel*goal_vel(i,0)/max;
    }
  }
  max=1.0;
  for(int i=3; i<6; i++){
    if(mask_cart[i]==1){
      if(std::abs(goal_vel(i,0))>max)
        max=std::abs(goal_vel(i,0));
    }
  }
  if(max>max_vel){
    for(int i=3; i<6; i++){
      goal_vel(i,0)=max_vel*goal_vel(i,0)/max;
    }
  }*/

  return goal_vel;
}

float calculateMaxNegativeVel(float current_joint, float min_joint_value, float acceleration, float sampling_duration){
  return std::min(std::max((min_joint_value-current_joint)/sampling_duration, -std::sqrt(2*acceleration*(current_joint-min_joint_value))),(float)0.0);
}
float calculateMaxPositiveVel(float current_joint, float max_joint_value, float acceleration, float sampling_duration){
  return std::max(std::min((max_joint_value-current_joint)/sampling_duration, std::sqrt(2*acceleration*(max_joint_value-current_joint))),(float)0.0);
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "task_priority");
  ros::NodeHandle nh;


  ros::Subscriber joints_sub= nh.subscribe("/uwsim/joint_state", 1, jointsCallback);
  ros::Publisher joints_pub=nh.advertise<sensor_msgs::JointState>("/uwsim/joint_state_command",1);
  ros::Publisher odom_pub=nh.advertise<nav_msgs::Odometry>("/dataNavigator", 1);
  tf::TransformListener listener;


  int n_joints=8;
  float acceleration=2;
  float max_joint_vel=0.5;
  float max_cart_vel=0.1;
  float sampling_duration=0.1;
  std::vector<float> max_joint_limit(n_joints,100), min_joint_limit(n_joints,-100);

  max_joint_limit[0]=100; min_joint_limit[0]=-100;  // x vehicle
  max_joint_limit[1]=100; min_joint_limit[1]=-100;  // y vehicle
  max_joint_limit[2]=100; min_joint_limit[2]=-100;  // z vehicle
  max_joint_limit[3]=50*M_PI/180; min_joint_limit[3]=-100;  // yaw vehicle
  max_joint_limit[4]=0.54; min_joint_limit[4]=-1.57;  // slew
  //max_joint_limit[4]=10*M_PI/180; min_joint_limit[4]=-1.57;
  max_joint_limit[5]=1.57; min_joint_limit[5]=0.0;  // shoulder
  max_joint_limit[6]=2.1529; min_joint_limit[6]=0.0;  // elbow
  max_joint_limit[7]=3.14; min_joint_limit[7]=-3.14;  // jaw






  KDL::Chain auv_arm, auv;

  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0,  0,  1.08    , M_PI     )));
  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 7.98*M_PI/180     )));
  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083,  M_PI_2,  0.0    , 113*M_PI/180     )));
  auv_arm.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.49938    , 0.0     )));

  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
  auv.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));

  std::vector<KDL::Chain> chains;
  chains.push_back(auv_arm);
  chains.push_back(auv);






//Task 1

  std::vector<int> mask_cartesian_1(6, 0);
  mask_cartesian_1[0]=1;
  mask_cartesian_1[1]=1;
  mask_cartesian_1[2]=1;
  mask_cartesian_1[3]=1;
  mask_cartesian_1[4]=1;
  mask_cartesian_1[5]=0;

  std::vector<int> chain_joint_relation_1;
  for(int i=0; i<8; i++){
    chain_joint_relation_1.push_back(i);
  }
  std::vector<int> mask_joint_1(8, 1);
  //mask_joint_1[7]=0;


 // Eigen::MatrixXd goal_1(6,1);
  //goal_1<< 3.5,1.3,2.5,120*M_PI/180,30*M_PI/180,180*M_PI/180;
 // GoalFixedPosePtr goal_class_1(new GoalFixedPose(goal_1, auv_arm, mask_cartesian_1, chain_joint_relation_1));
  GoalROSTwistPtr goal_class_1(new GoalROSTwist(mask_cartesian_1, "/commands", nh));
  TaskPtr task_1(new Task(auv_arm, mask_cartesian_1, n_joints, mask_joint_1, chain_joint_relation_1, goal_class_1));
  std::vector<TaskPtr> multiTask_1;
  multiTask_1.push_back(task_1);

  std::vector<std::vector<int> > joints_priority_1;
  std::vector<int> first_priority_1, second_priority_1;
  for(int i=4; i<8; i++){
    first_priority_1.push_back(i);
  }
  for(int i=0; i<4; i++){
    second_priority_1.push_back(i);
  }
  joints_priority_1.push_back(first_priority_1);
  joints_priority_1.push_back(second_priority_1);

   MultiTaskPtr multi_1(new MultiTask(multiTask_1, chains, joints_priority_1));





  //Task 2

   /* std::vector<int> mask_cartesian_2(6, 0);
    mask_cartesian_2[0]=0;
    mask_cartesian_2[1]=0;
    mask_cartesian_2[2]=0;
    mask_cartesian_2[3]=1;
    mask_cartesian_2[4]=1;
    mask_cartesian_2[5]=1;

    std::vector<int> chain_joint_relation_2;
    for(int i=0; i<8; i++){
      chain_joint_relation_2.push_back(i);
    }
    std::vector<int> mask_joint_2(8, 1);
    mask_joint_2[3]=0;


   // Eigen::MatrixXd goal_1(6,1);
    //goal_1<< 3.5,1.3,2.5,120*M_PI/180,30*M_PI/180,180*M_PI/180;
   // GoalFixedPosePtr goal_class_1(new GoalFixedPose(goal_1, auv_arm, mask_cartesian_1, chain_joint_relation_1));
    GoalROSTwistPtr goal_class_2(new GoalROSTwist(mask_cartesian_2, "/commands", nh));
    TaskPtr task_2(new Task(auv_arm, mask_cartesian_2, n_joints, mask_joint_2, chain_joint_relation_2, goal_class_2));

    std::vector<TaskPtr> multiTask_2;
    multiTask_2.push_back(task_2);
    MultiTaskPtr multi_2(new MultiTask(multiTask_2));
    std::vector<std::vector<int> > joints_priority_2;
    std::vector<int> first_priority_2, second_priority_2;
    for(int i=4; i<8; i++){
      first_priority_2.push_back(i);
    }
    for(int i=0; i<4; i++){
      second_priority_2.push_back(i);
    }
    joints_priority_2.push_back(first_priority_2);
    joints_priority_2.push_back(second_priority_2);*/






  //Task2

  std::vector<int> mask_cartesian_2(6, 0);
  mask_cartesian_2[0]=0;
  mask_cartesian_2[1]=0;
  mask_cartesian_2[2]=0;
  mask_cartesian_2[3]=0;
  mask_cartesian_2[4]=0;
  mask_cartesian_2[5]=1;

  std::vector<int> chain_joint_relation_2;
  for(int i=0; i<4; i++){
    chain_joint_relation_2.push_back(i);
  }
  std::vector<int> mask_joint_2(4, 0);
  mask_joint_2[3]=1;

  Eigen::MatrixXd goal_2(6,1);
  goal_2<< 0.0,0.0,0.0,0,0,0;
  GoalFixedPosePtr goal_class_2(new GoalFixedPose(goal_2, auv, mask_cartesian_2, chain_joint_relation_2));

  TaskPtr task_2(new Task(auv, mask_cartesian_2, n_joints, mask_joint_2, chain_joint_relation_2, goal_class_2));

  std::vector<TaskPtr> multiTask_2;
  multiTask_2.push_back(task_2);

  std::vector<std::vector<int> > joints_priority_2;
  std::vector<int> first_priority_2, second_priority_2;
  for(int i=0; i<4; i++){
    first_priority_2.push_back(i);
  }

  joints_priority_2.push_back(first_priority_2);
  MultiTaskPtr multi_2(new MultiTask(multiTask_2, chains, joints_priority_2));



  //Task3

 /* std::vector<int> mask_cartesian_3(6, 0);
  mask_cartesian_3[0]=1;
  mask_cartesian_3[1]=0;
  mask_cartesian_3[2]=0;
  mask_cartesian_3[3]=0;
  mask_cartesian_3[4]=0;
  mask_cartesian_3[5]=0;

  std::vector<int> chain_joint_relation_3;
  for(int i=0; i<8; i++){
    chain_joint_relation_3.push_back(i);
  }
  std::vector<int> mask_joint_3(8, 1);


  TaskPtr task_3(new Task(auv_arm, mask_cartesian_3, n_joints, mask_joint_3, chain_joint_relation_3));
  Eigen::MatrixXd goal_3(6,1);
  goal_3<< 2.2,0.0,2.0, 0.0, 0.0, 0.0;
  std::vector<TaskPtr> multiTask_3;
  multiTask_3.push_back(task_3);
  MultiTaskPtr multi_3(new MultiTask(multiTask_3));
  std::vector<std::vector<int> > joints_priority_3;
  std::vector<int> first_priority_3, second_priority_3;
  for(int i=0; i<4; i++){
    second_priority_3.push_back(i);
  }
  for(int i=4; i<8; i++){
    first_priority_3.push_back(i);
  }
  joints_priority_3.push_back(first_priority_3);
  joints_priority_3.push_back(second_priority_3);*/





  while(!joints_init && ros::ok()){
    ros::spinOnce();
    ros::Duration(sampling_duration).sleep();
  }



   std::vector<float> joints_auv(4,0);

  std::vector<float> odom(6,0);
  bool finished=false;
  while(!finished && ros::ok()){
    Eigen::MatrixXd T_k_complete;
    Eigen::MatrixXd vels(n_joints,1);
    vels<<0,0,0,0,0,0,0,0;
    ros::spinOnce();
    tf::StampedTransform transform;

    try{
      listener.lookupTransform("world", "girona500", ros::Time(0), transform);
      double roll, pitch, yaw;
      //tf::Matrix3x3(transform.getRotation()).getEulerZYX(yaw, pitch, roll);
      Eigen::Quaterniond rotation(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());

      /*rotation.x()=transform.getRotation().x();
      rotation.y()=transform.getRotation().y();
      rotation.z()=transform.getRotation().z();
      rotation.w()=transform.getRotation().w();*/
      Eigen::Vector3d euler=rotation.toRotationMatrix().eulerAngles(0,1,2);
      Eigen::Quaterniond ident;
      ident.setIdentity();

      //Eigen::Vector3d orientation=restaQuaternion(rotation, ident);
      roll=euler[0];
      pitch=euler[1];
      yaw=euler[2];


      odom[0]=transform.getOrigin().x();
      odom[1]=transform.getOrigin().y();
      odom[2]=transform.getOrigin().z();
      odom[3]=roll;
      odom[4]=pitch;
      odom[5]=yaw;


      std::vector<float>  current_joints;
      for(int i=0; i<joints_auv.size(); i++){
        current_joints.push_back(joints_auv[i]);
      }
      for(int i=0; i<joints_arm.size(); i++){
        current_joints.push_back(joints_arm[i]);
      }



      //calculate max positive and negative velocities
      std::vector<float> max_positive_velocities, max_negative_velocities, max_positive_cartesian_vel, max_negative_cartesian_vel;
      //std::cout<<"Max positive and negative velocities"<<std::endl;
      for(int i=0; i<current_joints.size(); i++){
        max_positive_velocities.push_back(calculateMaxPositiveVel(current_joints[i], max_joint_limit[i], acceleration, sampling_duration));
        max_negative_velocities.push_back(calculateMaxNegativeVel(current_joints[i], min_joint_limit[i], acceleration, sampling_duration));
      }
      //sleep(10);


     /* //Task3
      std::cout<<"Task3-----------------"<<std::endl;
      goal_vel=calculateGoalVel(goal_3, auv_arm, current_joints, odom, max_cart_vel, mask_cartesian_3);
      std::cout<<"goal_vel "<<goal_vel<<std::endl;
      multi_3->setGoal(0, goal_vel);
      multi_3->setCurrentJoints(current_joints);
      multi_3->setOdom(odom);
      multi_3->setMaxPositiveVelocity(max_positive_velocities);
      multi_3->setMaxNegativeVelocity(max_negative_velocities);
      vels=multi_3->calculateMultiTaskVel(vels,  T_k_complete);
      T_k_complete=multi_3->getT_k_complete();*/
      odom[0]=0;
      odom[1]=0;
      odom[2]=0;
      odom[3]=0;
      odom[4]=0;
      odom[5]=0;

      //Task2
      std::cout<<"Task2-----------------"<<std::endl;
      //Eigen::MatrixXd goal_vel2=calculateGoalVel(goal_2, auv_arm, current_joints, odom, max_cart_vel, mask_cartesian_2);
      //multi_2->setGoal(0, goal_vel2);
      multi_2->setCurrentJoints(current_joints);
      multi_2->setOdom(odom);
      multi_2->setMaxPositiveJointVelocity(max_positive_velocities);
      multi_2->setMaxNegativeJointVelocity(max_negative_velocities);
      vels=multi_2->calculateMultiTaskVel(vels, T_k_complete);
      T_k_complete=multi_2->getT_k_complete();


      Eigen::MatrixXd direction=task_2->getJacobianNoMask()*vels;
      std::cout<<"JAcobian"<<std::endl;
      std::cout<<task_2->getJacobianNoMask()<<std::endl;
      std::cout<<"vels"<<std::endl;
      std::cout<<vels<<std::endl;
      std::cout<<"Direcction<<<<<<<<"<<std::endl;
      for(int i=0; i< 6; i++){
        if(direction(i,0)<0.00001 && direction(i,0)>-0.00000001){
          direction(i,0)=0;
        }

      }
      std::cout<<direction(0,0)<<std::endl;
      std::cout<<direction(1,0)<<std::endl;
      std::cout<<direction(2,0)<<std::endl;
      std::cout<<direction(3,0)<<std::endl;
      std::cout<<direction(4,0)<<std::endl;
      std::cout<<direction(5,0)<<std::endl;





      //Task1
      std::cout<<"Task1-----------------"<<std::endl;
      //Eigen::MatrixXd goal_vel1=calculateGoalVel(goal_1, auv_arm, current_joints, odom, max_cart_vel, mask_cartesian_1);
      //multi_1->setGoal(0, goal_vel1);
      multi_1->setCurrentJoints(current_joints);
      multi_1->setOdom(odom);
      multi_1->setMaxPositiveJointVelocity(max_positive_velocities);
      multi_1->setMaxNegativeJointVelocity(max_negative_velocities);
      vels=multi_1->calculateMultiTaskVel(vels, T_k_complete);
      T_k_complete=multi_1->getT_k_complete();


      float max_vel=0;
      for(int i=0; i<vels.rows(); i++){
        if(std::abs(vels(i,0))>max_vel){
          max_vel=std::abs(vels(i,0));
        }
      }
      if(max_vel>max_joint_vel){
        for(int i=0; i<vels.rows(); i++){
          vels(i,0)=vels(i,0)*max_joint_vel/max_vel;
        }
      }


      direction=task_1->getJacobianNoMask()*vels;
      std::cout<<"JAcobian"<<std::endl;
      std::cout<<task_1->getJacobianNoMask()<<std::endl;
      std::cout<<"vels"<<std::endl;
      std::cout<<vels<<std::endl;
      std::cout<<"Direcction<<<<<<<<"<<std::endl;
      for(int i=0; i< 6; i++){
        if(direction(i,0)<0.00001 && direction(i,0)>-0.00000001){
          direction(i,0)=0;
        }

      }
      std::cout<<direction(0,0)<<std::endl;
      std::cout<<direction(1,0)<<std::endl;
      std::cout<<direction(2,0)<<std::endl;
      std::cout<<direction(3,0)<<std::endl;
      std::cout<<direction(4,0)<<std::endl;
      std::cout<<direction(5,0)<<std::endl;






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

      odom_pub.publish(odom_msg);
      joints_pub.publish(joint_msg);
      ros::spinOnce();


    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s\n", ex.what());
      ros::Duration(sampling_duration).sleep();
    }
   ros::Duration(sampling_duration).sleep();
  }



  return 0;
}
