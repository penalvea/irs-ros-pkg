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






MultiTask::MultiTask(std::vector<TaskPtr> tasks, std::vector<KDL::Chain> chains, std::vector<std::vector<int> > chain_joint_relations, std::vector<std::vector<int> > joints_priority){
  tasks_=tasks;
  chains_=chains;
  chain_joint_relations_=chain_joint_relations;
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
  std::cout<<"Limit cartesian velocities"<<std::endl;
  for(int j=0; j<chains_.size(); j++){
    std::cout<<"Chain "<<j<<std::endl;

    for(int i=0; i< max_positive_cartesian_vel_[j].size(); i++){
      std::cout<<max_positive_cartesian_vel_[j][i]<<"     "<<max_negative_cartesian_vel_[j][i]<<std::endl;
    }
  }
  std::cout<<"Limit joint velocities"<<std::endl;
  for(int i=0; i< max_positive_joint_vel_.size(); i++){
    std::cout<<max_positive_joint_vel_[i]<<"     "<<max_negative_joint_vel_[i]<<std::endl;
  }

  Eigen::MatrixXd jac_joint(vels.rows(), vels.rows()), desired_vel_joint(vels.rows(),1);
  jac_joint.setZero();
  for(int i=0; i<vels.rows(); i++){
    jac_joint(i,i)=1;
  }
  desired_vel_joint.setZero();

  Eigen::MatrixXd jac_cartesian(chains_.size()*6, vels.rows()),desired_vel_cartesian(chains_.size()*6,1);
  jac_cartesian.setZero();
  desired_vel_cartesian.setZero();

  for(int i=0; i<chains_.size(); i++){
    KDL::Chain chain_odom;
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY)));
    chain_odom.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    chain_odom.addChain(chains_[i]);
    KDL::ChainJntToJacSolver jac_sovler(chain_odom);

    KDL::JntArray q(chain_odom.getNrOfJoints());
    for(int j=0; j<odom_.size(); j++){
      q(j)=odom_[j];
    }
    for(int j=0; j<chains_[i].getNrOfJoints(); j++){
      q(j+odom_.size())=current_joints_[chain_joint_relations_[i][j]];
    }

    KDL::Jacobian jac_chain(chain_odom.getNrOfJoints());
    jac_sovler.JntToJac(q, jac_chain);

    for(int j=0; j<chain_joint_relations_[i].size(); j++){
     jac_cartesian(i*6,chain_joint_relations_[i][j])=jac_chain(0, j+odom_.size());
     jac_cartesian(i*6+1,chain_joint_relations_[i][j])=jac_chain(1, j+odom_.size());
     jac_cartesian(i*6+2,chain_joint_relations_[i][j])=jac_chain(2, j+odom_.size());
     jac_cartesian(i*6+3,chain_joint_relations_[i][j])=jac_chain(3, j+odom_.size());
     jac_cartesian(i*6+4,chain_joint_relations_[i][j])=jac_chain(4, j+odom_.size());
     jac_cartesian(i*6+5,chain_joint_relations_[i][j])=jac_chain(5, j+odom_.size());
    }
  }



  bool finished=false;
  bool limit_cartesian=false;
  std::vector<int> active_cartesians(chains_.size()*6,0);
  std::vector<int> active_joints(vels.rows(),0);
  while(!finished){
    finished=true;
    limit_cartesian=false;


    Eigen::MatrixXd cartesian_velocity=jac_cartesian*vels;
    for(int i=0; i<chains_.size(); i++){
      for(int j=0; j<6; j++){
          if(cartesian_velocity(i*6+j,0)>max_positive_cartesian_vel_[i][j]+0.000001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_positive_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);
            //std::cout<<"desired vel cartesian"<<std::endl;
            //std::cout<<desired_vel_cartesian<<std::endl;
            finished=false;
            limit_cartesian=true;
            std::fill(active_joints.begin(), active_joints.end(), 0);
            std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" mayor que max_positive"<<std::endl;
            std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_positive_cartesian_vel_[i][j]<<std::endl;
          }
          else if(cartesian_velocity(i*6+j,0)<max_negative_cartesian_vel_[i][j]-0.000001){
            active_cartesians[i*6+j]=1;
            desired_vel_cartesian(i*6+j,0)=max_negative_cartesian_vel_[i][j]-cartesian_velocity(i*6+j);

            finished=false;
            limit_cartesian=true;
            std::fill(active_joints.begin(), active_joints.end(), 0);
            std::cout<<"Cartesian chain "<<i<<" axis "<<j<<" menor que max_negative"<<std::endl;
            std::cout<<cartesian_velocity(i*6+j)<<"    ----->  "<<max_negative_cartesian_vel_[i][j]<<std::endl;
          }
          else if(active_cartesians[i*6+j]==1){
            desired_vel_cartesian(i*6+j,0)=0;
        }
      }
    }

    if(!limit_cartesian){
      for(int i=0; i<vels.rows(); i++){
        if(vels(i,0)>max_positive_joint_vel_[i]+0.000001){
          //jac_joint(i,i)=1;
          active_joints[i]=1;
          desired_vel_joint(i,0)=max_positive_joint_vel_[i]-vels(i,0);
          finished=false;
          std::cout<<"Joint "<<i<<"mayor que max_positive"<<std::endl;
          std::cout<<vels(i,0)<<"    ----->  "<<max_positive_joint_vel_[i]<<std::endl;
        }
        else if(vels(i,0)<max_negative_joint_vel_[i]-0.000001){
          //jac_joint(i,i)=1;
          active_joints[i]=1;
          desired_vel_joint(i,0)=max_negative_joint_vel_[i]-vels(i,0);
          finished=true;
          std::cout<<"Joint "<<i<<"mayor que max_negative"<<std::endl;
          std::cout<<vels(i,0)<<"    ----->  "<<max_negative_joint_vel_[i]<<std::endl;
        }
        else if(active_joints[i]==1){
          desired_vel_joint(i,0)=0;
        }
      }
    }

    Eigen::MatrixXd jac_cartesian_modified=jac_cartesian;
    Eigen::MatrixXd jac_joint_modified=jac_joint;

    for(int i=0; i<active_cartesians.size(); i++){
      if(active_cartesians[i]==0){
        jac_cartesian_modified.row(i).setZero();
      }
    }
    for(int i=0; i<active_joints.size(); i++){
      if(active_joints[i]==0){
        jac_joint_modified.row(i).setZero();
      }
    }


    Eigen::MatrixXd new_T_k_complete(jac_cartesian_modified.rows()+jac_joint_modified.rows()+T_k_complete_.rows(), jac_joint_modified.cols());
    new_T_k_complete<<jac_cartesian_modified, jac_joint_modified,T_k_complete_;
    Eigen::MatrixXd new_T_k_inverse=pinvMat(new_T_k_complete);
    Eigen::MatrixXd new_T_k(jac_joint_modified.cols(), jac_cartesian_modified.rows()+jac_joint_modified.rows());
    for(int i=0; i<new_T_k.rows(); i++){
      for(int j=0; j<new_T_k.cols(); j++){
        new_T_k(i,j)=new_T_k_inverse(i,j);
      }
    }



    Eigen::MatrixXd jac(jac_cartesian_modified.rows()+jac_joint_modified.rows(), jac_joint_modified.cols());
    jac<<jac_cartesian_modified, jac_joint_modified;
    Eigen::MatrixXd desired_vel(desired_vel_cartesian.rows()+ desired_vel_joint.rows(),1);
    desired_vel<<desired_vel_cartesian, desired_vel_joint;

    /*std::cout<<"-----------------------------------------------"<<std::endl;
    std::cout<<vels<<std::endl;
    std::cout<<desired_vel<<std::endl;

    std::cout<<(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel<<std::endl;

    std::cout<<"-----------------------------------------------"<<std::endl;*/


    vels=vels+(new_T_k*(pinvMat(jac*new_T_k)))*desired_vel;
   /* std::cout<<"new vel"<<std::endl;
    std::cout<<vels<<std::endl;*/


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
