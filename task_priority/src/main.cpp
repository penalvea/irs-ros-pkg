#include <ros/ros.h>
#include <kdl/chain.hpp>
#include <string>
#include <boost/lexical_cast.hpp>
#include "task_priority/controller.hpp"


int main(int argc, char **argv){
  ros::init(argc, argv, "multiple_task_priority_control");
  ros::NodeHandle nh;

  double acceleration, max_joint_vel, max_cartesian_vel, sampling_duration;
  nh.getParam("acceleration", acceleration);
  ROS_INFO("Acceleration: %f", acceleration);
  nh.getParam("max_joint_vel", max_joint_vel);
  ROS_INFO("Max joint vel: %f", max_joint_vel);
  nh.getParam("max_cartesian_vel", max_cartesian_vel);
  ROS_INFO("Max cartesian vel: %f", max_cartesian_vel);
  nh.getParam("sampling_duration", sampling_duration);
  ROS_INFO("Sampling duration: %f", sampling_duration);

  int n_joints;
  nh.getParam("n_joints", n_joints);

 std::vector<float> max_joint_limit, min_joint_limit;
  nh.getParam("max_joint_limit", max_joint_limit);
  nh.getParam("min_joint_limit", min_joint_limit);

  if(max_joint_limit.size()!=n_joints){
    ROS_ERROR("max joint limit must have %d limits", n_joints);
    return -1;
  }
  if(min_joint_limit.size()!=n_joints){
    ROS_ERROR("min joint limit must have %d limits", n_joints);
    return -1;
  }


  ////// Chains
  ///
  ///

  std::vector<std::string> chain_names;
  nh.getParam("chain_names", chain_names);

  std::map<std::string, int> chain_id;
  std::vector<KDL::Chain> chains;
  std::vector<std::vector<int> > chain_joint_relations;
  std::vector<std::vector<float> > max_cartesian_limits, min_cartesian_limits;
  for(int i=0; i<chain_names.size(); i++){

    chain_id[chain_names[i]]=i;
    KDL::Chain chain;
    bool defined=false;
    int cont=1;
    while(!defined){
      XmlRpc::XmlRpcValue definition;
      if(nh.getParam(chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont), definition)){
        cont++;
        if( definition.size()!=1 && definition.size()!=5){

          ROS_ERROR("%s must have 1 or 5 elements", (chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont)).c_str());
          return -1;
        }
        std::string joint_mov=static_cast<std::string>(definition[0]);
        if(definition.size()==1){
          if(joint_mov=="TransX"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
          }
          else if(joint_mov=="TransY"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
          }
          else if(joint_mov=="TransZ"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
          }
          else if(joint_mov=="RotX"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
          }
          else if(joint_mov=="RotY"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
          }
          else if(joint_mov=="RotZ"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
          }
          else{
            ROS_ERROR("%s: incorrect rotation and translation", (chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont)).c_str());
            return -1;
          }
        }
        else{
          double theta, d, a, alpha;
          a=static_cast<double>(definition[1]);
          alpha=static_cast<double>(definition[2]);
          d=static_cast<double>(definition[3]);
          theta=static_cast<double>(definition[4]);
          if(joint_mov=="TransX"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX), KDL::Frame().DH(a, alpha, d, theta)));
          }
          else if(joint_mov=="TransY"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY), KDL::Frame().DH(a, alpha, d, theta)));
          }
          else if(joint_mov=="TransZ"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ), KDL::Frame().DH(a, alpha, d, theta)));
          }
          else if(joint_mov=="RotX"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), KDL::Frame().DH(a, alpha, d, theta)));
          }
          else if(joint_mov=="RotY"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), KDL::Frame().DH(a, alpha, d, theta)));
          }
          else if(joint_mov=="RotZ"){
            chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH(a, alpha, d, theta)));
          }
          else{
            ROS_ERROR("%s: incorrect rotation and translation", (chain_names[i]+"/joint_"+boost::lexical_cast<std::string>(cont)).c_str());
            return -1;
          }

        }
      }
      else{
        defined=true;
        chains.push_back(chain);
        std::vector<int> chain_joint_relation;
        if(!nh.getParam(chain_names[i]+"/chain_joint_relation", chain_joint_relation)){
          ROS_ERROR("Every chain need the chain_joint_relation");
          return -1;
        }
        chain_joint_relations.push_back(chain_joint_relation);

        std::vector<float> max_cartesian_limit, min_cartesian_limit;
        if(!nh.getParam(chain_names[i]+"/max_cartesian_limit", max_cartesian_limit)){
          ROS_ERROR("Every chain need the max_cartesian_limit");
          return -1;
        }

        max_cartesian_limits.push_back(max_cartesian_limit);
        if(!nh.getParam(chain_names[i]+"/min_cartesian_limit", min_cartesian_limit)){
          ROS_ERROR("Every chain need the min_cartesian_limit");
          return -1;
        }
        min_cartesian_limits.push_back(min_cartesian_limit);

        ROS_INFO("chain %s added: %d joints", chain_names[i].c_str(), cont-1);
      }
    }

   }


  ////// MultiTask
  ///
  ///

  std::vector<std::string> multitask_priority;
  nh.getParam("multitask_priority", multitask_priority);


  std::vector<MultiTaskPtr> multitasks;
  for(int i=0; i< multitask_priority.size(); i++){
    std::vector<std::string> task_names;

    nh.getParam(multitask_priority[i]+"/tasks", task_names);
    ROS_INFO("Multitask: %s", multitask_priority[i].c_str());
    std::vector<std::vector<int> > joint_priority;

    bool finished=false;
    int cont=1;
    while(!finished){
      std::vector<int> individual_joint_priority;
      if(nh.getParam(multitask_priority[i]+"/joint_priority_"+boost::lexical_cast<std::string>(cont), individual_joint_priority)){
        joint_priority.push_back(individual_joint_priority);
        cont++;
      }
      else{
        finished=true;
      }

    }
    ROS_INFO("    %d joint_priorities", cont-1);
    std::vector<TaskPtr> tasks;
    for(int j=0; j<task_names.size(); j++){
      ROS_INFO("    Task: %s", task_names[j].c_str());
      std::string task_chain;
      nh.getParam(task_names[j]+"/chain", task_chain);
      ROS_INFO("        Chain: %s", task_chain.c_str());
      bool cartesian;
      nh.getParam(task_names[j]+"/cartesian", cartesian);
      ROS_INFO("        Cartesian: %d", cartesian);
      std::vector<int> task_mask_cartesian, task_mask_joint;
      nh.getParam(task_names[j]+"/mask_cartesian", task_mask_cartesian);
      std::string msg;
      for(int k=0; k<task_mask_cartesian.size(); k++){
        msg+=boost::lexical_cast<std::string>(task_mask_cartesian[k])+" ";
      }
      ROS_INFO("        Mask Cartesian: %s", msg.c_str());
      nh.getParam(task_names[j]+"/mask_joint", task_mask_joint);
      msg="";
      for(int k=0; k<task_mask_joint.size(); k++){
        msg+=boost::lexical_cast<std::string>(task_mask_joint[k])+" ";
      }
      ROS_INFO("        Mask Joint: %s", msg.c_str());
      std::string goal_type;
      nh.getParam(task_names[j]+"/goal_type", goal_type);
      GoalPtr goal;
      if(goal_type=="Fixed"){
        ROS_INFO("        Goal: Fixed");
        std::vector<float> desired_pose;
        nh.getParam(task_names[j]+"/goal/desired_pose", desired_pose);
        Eigen::MatrixXd desired_pose_eigen(6,1);
        for(int k=0; k<6; k++){
          desired_pose_eigen(k,0)=desired_pose[k];
        }
        msg="";
        for(int k=0; k<desired_pose.size(); k++){
          msg+=boost::lexical_cast<std::string>(desired_pose[k])+" ";
        }
        ROS_INFO("            Desired Pose: %s", msg.c_str());
        GoalFixedPosePtr goal_fixed(new GoalFixedPose(desired_pose_eigen, chains[chain_id[task_chain]], task_mask_cartesian, chain_joint_relations[chain_id[task_chain]], max_cartesian_vel));
        goal=goal_fixed;
      }
      else if(goal_type=="ROS_Pose"){
        ROS_INFO("        Goal: ROS Pose");
        std::string goal_topic;
        nh.getParam(task_names[j]+"/goal/topic", goal_topic);
        ROS_INFO("            ROS Node: %s", goal_topic.c_str());
        GoalROSPosePtr goal_ros_pose(new GoalROSPose(chains[chain_id[task_chain]], task_mask_cartesian, goal_topic, nh, chain_joint_relations[chain_id[task_chain]], max_cartesian_vel));
        goal=goal_ros_pose;
      }
      else if(goal_type=="ROS_Twist"){
        ROS_INFO("        Goal: ROS Twist");
        std::string goal_topic;
        nh.getParam(task_names[j]+"/goal/topic", goal_topic);
        ROS_INFO("            ROS Node: %s", goal_topic.c_str());
        GoalROSTwistPtr goal_ros_twist(new GoalROSTwist(task_mask_cartesian, goal_topic, nh, max_cartesian_vel));
        goal=goal_ros_twist;
      }
      else if(goal_type=="Joints"){
        ROS_INFO("        Goal: Joints");
        std::vector<float> joints_position;
        nh.getParam(task_names[j]+"/goal/joints_position", joints_position);
        msg="";
        for(int k=0; k<joints_position.size(); k++){
          msg+=boost::lexical_cast<std::string>(joints_position[k])+" ";
        }
        ROS_INFO("            Desired Joints Position: %s", msg.c_str());
        GoalJointsPositionPtr goal_joint(new GoalJointsPosition(joints_position));
        goal=goal_joint;
      }
      else{
        ROS_ERROR("Goal must be Fixed, ROS_Pose, ROS_Twist or Joints");
      }
      bool frame_inertial;
      nh.getParam(task_names[j]+"/frame_inertial", frame_inertial);
      ROS_INFO("        Frame Inertial: %d", frame_inertial);
      TaskPtr task(new Task(chains[chain_id[task_chain]],task_mask_cartesian, n_joints, task_mask_joint, chain_joint_relations[chain_id[task_chain]], goal, frame_inertial, cartesian, task_names[j] ));
      tasks.push_back(task);

    }
  MultiTaskPtr multi(new MultiTask(tasks, chains, chain_joint_relations, joint_priority, multitask_priority[i]));
  multitasks.push_back(multi);
  }


  std::string arm_joint_state_topic, arm_joint_command_topic, vehicle_tf, world_tf, vehicle_command_topic;
  nh.getParam("arm_joint_state_topic", arm_joint_state_topic);
  ROS_INFO("Joint State Topi: %s", arm_joint_state_topic.c_str());
  nh.getParam("arm_joint_command_topic", arm_joint_command_topic);
  ROS_INFO("Command Joint Topic: %s", arm_joint_state_topic.c_str());
  nh.getParam("vehicle_tf", vehicle_tf);
  ROS_INFO("Vehicle tf: %s", vehicle_tf.c_str());
  nh.getParam("world_tf", world_tf);
  ROS_INFO("World tf: %s", world_tf.c_str());
  nh.getParam("vehicle_command_topic", vehicle_command_topic);
  ROS_INFO("Command Vehicle Topic: %s", vehicle_command_topic.c_str());

  ControllerPtr controller(new Controller(multitasks, n_joints, max_joint_limit, min_joint_limit, max_cartesian_limits, min_cartesian_limits, acceleration, max_joint_vel, sampling_duration, nh, arm_joint_state_topic, arm_joint_command_topic, vehicle_tf, world_tf, vehicle_command_topic, chains, chain_joint_relations));
  controller->goToGoal();


  return 0;
}
