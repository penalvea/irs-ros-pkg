#include "task_priority/task.hpp"

Task::Task(const KDL::Chain &chain, std::vector<int> mask_cartesian, int n_joints, std::vector<int> mask_joint, std::vector<int> chain_joint_relation, GoalPtr goal, bool frame_inertial){
  active_=true;
  chain_=chain;
  chain_joint_relation_=chain_joint_relation;
  mask_cartesian_=mask_cartesian;
  mask_joint_=mask_joint;
  jac_.reset(new CartesianJacobian(chain, n_joints, chain_joint_relation, mask_cartesian, mask_joint, frame_inertial));
  task_velocity_.reset(new CartesianTaskVelocity());
  goal_=goal;

}

Task::~Task(){}

void Task::activate(){
  active_=true;
}
void Task::deactivate(){
  active_=false;
}
bool Task::isActive(){
  return active_;
}


Eigen::MatrixXd Task::calculateCartesianError(Eigen::MatrixXd current_joint_vel, std::vector<float> joints, std::vector<float> odom){
  /*std::cout<<"current direction****"<<std::endl;
  std::cout<<jac_->getJacNoMask()*current_joint_vel<<std::endl;
  std::cout<<"goal---"<<std::endl;
  std::cout<<goal_<<std::endl;*/

  Eigen::MatrixXd vel_error=task_velocity_->calculateCartesianVelocity(jac_->getJacNoMask()*current_joint_vel, goal_->getGoal(joints, odom));
  //std::cout<<vel_error<<std::endl;

  for(int i=0; i<vel_error.size(); i++){
    if(mask_cartesian_[i]==0){

      vel_error(i,0)=0;
        }
  }
  return vel_error;

}

Eigen::MatrixXd Task::getJacobian(){
  return jac_->getJac();
}
Eigen::MatrixXd Task::getJacobianNoMask(){
  return jac_->getJacNoMask();
}
void Task::calculateJacobian(std::vector<float> current_joints, std::vector<float> odom){
  jac_->setOdom(odom);
  jac_->calculateJac(current_joints);
}


bool Task::goalInitialized(){
  return goal_->getInitialized();
}






MultiTask::MultiTask(std::vector<TaskPtr> tasks, std::vector<KDL::Chain> chains, std::vector<std::vector<int> > joints_priority){
  tasks_=tasks;
  chains_=chains;
  joints_priority_=joints_priority;

}
MultiTask::~MultiTask(){}



Eigen::MatrixXd MultiTask::pinvMat(Eigen::MatrixXd matrix){
  double epsilon=std::numeric_limits<double>::epsilon();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance=epsilon*std::max(matrix.cols(), matrix.rows())*svd.singularValues().array().abs()(0);
  return svd.matrixV()*(svd.singularValues().array().abs()>tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal()*svd.matrixU().adjoint();

}

void MultiTask::setCurrentJoints(std::vector<float> joints){
  current_joints_=joints;
}

void MultiTask::calculateJacobians(Eigen::MatrixXd T_k_complete){
  for(int i=0; i<tasks_.size(); i++){
    tasks_[i]->calculateJacobian(current_joints_, odom_);
  }
  J_k_=tasks_[0]->getJacobian();
  J_k_no_mask_=tasks_[0]->getJacobianNoMask();
  for(int i=1; i<tasks_.size(); i++){
    Eigen::MatrixXd aux=J_k_;
    Eigen::MatrixXd aux_no_mask=J_k_no_mask_;
    Eigen::MatrixXd J_k_i=tasks_[i]->getJacobian();
    Eigen::MatrixXd J_k_i_no_mask=tasks_[i]->getJacobianNoMask();
    J_k_.resize(aux.rows()+J_k_i.rows(), aux.cols());
    J_k_<<aux,J_k_i;
    J_k_no_mask_.resize(aux_no_mask.rows()+J_k_i_no_mask.rows(), aux_no_mask.cols());
    J_k_no_mask_<<aux_no_mask,J_k_i_no_mask;
  }

  T_k_complete_.resize(T_k_complete.rows()+J_k_.rows(),J_k_.cols());
  T_k_complete_<<J_k_,T_k_complete;

}

void MultiTask::adaptJacobiansTask(std::vector<int> joints_active){
  Eigen::MatrixXd T_k_complete_task=T_k_complete_;
  Eigen::MatrixXd J_k_task=J_k_;
  for(int j=0; j<joints_active.size(); j++){
    if(joints_active[j]==0){
      for(int i=0; i<6; i++){
        T_k_complete_task(i,j)=0;
        J_k_task(i,j)=0;
      }
    }
  }




  Eigen::MatrixXd T_k_complete_task_inverse=pinvMat(T_k_complete_task);
  T_k_.resize(J_k_.cols(), J_k_.rows());
  for(int i=0; i<T_k_.rows(); i++){
    for(int j=0; j<T_k_.cols(); j++){
      T_k_(i,j)=T_k_complete_task_inverse(i,j);
    }
  }
  /*std::cout<<"Jacobiana"<<std::endl;
  std::cout<<J_k_<<std::endl;
  std::cout<<"T_k_complete"<<std::endl;
  std::cout<<T_k_complete_<<std::endl;
  std::cout<<"T_k_complete_task"<<std::endl;
  std::cout<<T_k_complete_task<<std::endl;
  std::cout<<"T_k_complete_task_inverse"<<std::endl;
  std::cout<<T_k_complete_task_inverse<<std::endl;
  std::cout<<"T_k_"<<std::endl;
  std::cout<<T_k_<<std::endl;*/




  J_null_inv_=T_k_*(pinvMat(J_k_task*T_k_));
  /*std::cout<<"J_null_inv_"<<std::endl;
  std::cout<<J_null_inv_<<std::endl;*/

}

Eigen::MatrixXd MultiTask::calculateError(Eigen::MatrixXd last_vel){
  Eigen::MatrixXd error=tasks_[0]->calculateCartesianError(last_vel, current_joints_, odom_);
  Eigen::MatrixXd aux;
  for(int i=1; i<tasks_.size(); i++){
    aux=error;
    Eigen::MatrixXd task_error=tasks_[i]->calculateCartesianError(last_vel, current_joints_, odom_);
    error.resize(aux.rows()+task_error.rows(), task_error.cols());
    error<<aux,task_error;
  }
  return error;
}

Eigen::MatrixXd MultiTask::calculateJointsVel(Eigen::MatrixXd error, std::vector<int> joints_active){
  Eigen::MatrixXd J_null_inv_adapted=J_null_inv_;
  return J_null_inv_adapted*error;
}

Eigen::MatrixXd MultiTask::calculateMultiTaskVel(Eigen::MatrixXd last_vel, Eigen::MatrixXd T_k_complete){
  std::cout<<" n_tareas= "<<tasks_.size()<<std::endl;
  ros::spinOnce();
  calculateJacobians(T_k_complete);
  std::cout<<"Joint priority size= "<<joints_priority_.size()<<std::endl;
  for(int i=0; i<joints_priority_.size(); i++){
    std::vector<int> joints_active(current_joints_.size(),0);
    for(int k=0; k<=i; k++){
      for(int j=0; j<joints_priority_[k].size(); j++){
        joints_active[joints_priority_[k][j]]=1;
      }
    }

    adaptJacobiansTask(joints_active);
    Eigen::MatrixXd error=calculateError(last_vel);
   // std::cout<<"El error cartesiano es"<<std::endl;
    //std::cout<<error<<std::endl;
    Eigen::MatrixXd next_vel=last_vel+calculateJointsVel(error, joints_active);
    //std::cout<<"Vels antes limitar"<<std::endl;
    //std::cout<<next_vel<<std::endl;
    next_vel=limitJointsAndCartesian(next_vel);
    //std::cout<<"Vels despues limitar"<<std::endl;
    //std::cout<<next_vel;
    last_vel=next_vel;
  }


    //std::cout<<"Cartesian Direction"<<std::endl;
    //std::cout<<calculateError(last_vel)<<std::endl;
    //std::cout<<"---------"<<std::endl;
    return last_vel;
  }
void MultiTask::setOdom(std::vector<float> odom){
  odom_=odom;
}
Eigen::MatrixXd MultiTask::getT_k_complete(){
  return T_k_complete_;
}

void MultiTask::setMaxPositiveJointVelocity(std::vector<float> vels){
  max_positive_joint_vel_=vels;
}
void MultiTask::setMaxNegativeJointVelocity(std::vector<float> vels){
  max_negative_joint_vel_=vels;
}
void MultiTask::setMaxPositiveCartesianVelocity(std::vector<std::vector<float> > vels){
  max_positive_cartesian_vel_=vels;
}
void MultiTask::setMaxNegativeCartesianVelocity(std::vector<std::vector<float> > vels){
  max_negative_cartesian_vel_=vels;
}

Eigen::MatrixXd MultiTask::limitJointsAndCartesian(Eigen::MatrixXd vels){
  Eigen::MatrixXd jac(vels.rows(), vels.rows()), desired_vel(vels.rows(),1);
  jac.setZero();
  desired_vel.setZero();
  for(int i=0; i< max_positive_joint_vel_.size(); i++){
    std::cout<<max_positive_joint_vel_[i]<<"     "<<max_negative_joint_vel_[i]<<std::endl;
  }


  bool finished=false;
  while(!finished){
    finished=true;
    for(int i=0; i<vels.rows(); i++){
      if(vels(i,0)>max_positive_joint_vel_[i]+0.000001){
        jac(i,i)=1;
        desired_vel(i,0)=max_positive_joint_vel_[i]-vels(i,0);
        finished=false;
        std::cout<<"Joint "<<i<<"mayor que max_positive"<<std::endl;
        std::cout<<vels(i,0)<<"    ----->  "<<max_positive_joint_vel_[i]<<std::endl;
      }
      if(vels(i,0)<max_negative_joint_vel_[i]-0.000001){
        jac(i,i)=1;
        desired_vel(i,0)=max_negative_joint_vel_[i]-vels(i,0);
        finished=true;
        std::cout<<"Joint "<<i<<"mayor que max_negative"<<std::endl;
        std::cout<<vels(i,0)<<"    ----->  "<<max_negative_joint_vel_[i]<<std::endl;
      }
    }

    Eigen::MatrixXd new_T_k_complete(jac.rows()+T_k_complete_.rows(), jac.cols());
    new_T_k_complete<<jac,T_k_complete_;
    Eigen::MatrixXd new_T_k_inverse=pinvMat(new_T_k_complete);
    Eigen::MatrixXd new_T_k(jac.cols(), jac.rows());
    for(int i=0; i<new_T_k.rows(); i++){
      for(int j=0; j<new_T_k.cols(); j++){
        new_T_k(i,j)=new_T_k_inverse(i,j);
      }
    }



    vels=vels+(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel;

  }


  return vels;
}


bool MultiTask::goalsInitialized(){
  bool initialized=true;
  for(int i=0; i<tasks_.size(); i++){
    initialized= initialized && tasks_[i]->goalInitialized();
  }
  return initialized;
}
