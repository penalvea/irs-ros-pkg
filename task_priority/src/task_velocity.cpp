#include <task_priority/task_velocity.hpp>




CartesianTaskVelocity::CartesianTaskVelocity(){}

CartesianTaskVelocity::~CartesianTaskVelocity(){}


Eigen::MatrixXd CartesianTaskVelocity::calculateCartesianVelocity(Eigen::MatrixXd current, Eigen::MatrixXd goal){
  Eigen::MatrixXd task_velocity(6,1);
  if(current.rows()!=6 || current.cols()!=1 || goal.rows()!=6 || goal.cols()!=1){
    std::cerr<<"current and goal vectors must have 6x1 elements"<<std::endl;
    std::cerr<<"Current="<<current<<std::endl;
    std::cerr<<"Goal="<<goal<<std::endl;
  }
  else{
    for(int i=0; i<3; i++){
      task_velocity(i,0)=goal(i,0)-current(i,0);
    }
    for(int i=3; i<6; i++){
      if(current(i,0)>M_PI)
        current(i,0)=-(2*M_PI)+current(i,0);
      else if(current(i,0)<-M_PI)
        current(i,0)=(2*M_PI)+current(i,0);
      if(goal(i,0)>M_PI)
        goal(i,0)= -(2*M_PI)+goal(i,0);
      else if(goal(i,0)<-M_PI)
        goal(i,0)=(2*M_PI)+goal(i,0);
      task_velocity(i,0)=goal(i,0)-current(i,0);
    }
  }
  return task_velocity;
}

Eigen::MatrixXd CartesianTaskVelocity::calculateJointVelocity(Eigen::MatrixXd current, Eigen::MatrixXd goal){
  return goal-current;
}
