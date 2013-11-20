#include "SimulatedPA10.h"


//Visp
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>


SimulatedPA10::SimulatedPA10(ros::NodeHandle &nh)
{
	qmax.resize(7);
	qmin.resize(7);
	qrange.resize(7);
	qamax.resize(7);
	qamin.resize(7);

	qmax[0]=177*M_PI/180;
	qmax[1]=91*M_PI/180;
	qmax[2]=174*M_PI/180;
	qmax[3]=137*M_PI/180;
	qmax[4]=255*M_PI/180;
	qmax[5]=165*M_PI/180;
	qmax[6]=360*M_PI/180;
	
	qmin[0]=-177*M_PI/180;
	qmin[1]=-91*M_PI/180;
	qmin[2]=-174*M_PI/180;
	qmin[3]=-137*M_PI/180;
	qmin[4]=-255*M_PI/180;
	qmin[5]=-165*M_PI/180;
	qmin[6]=-360*M_PI/180;

	qrange=qmax-qmin;
	qamax=qmax-0.1*qrange;
	qamin=qmin+0.1*qrange;
	qamax[3]=qmax[3]-0.2*qrange[3];
	qamin[3]=qmin[3]+0.7*qrange[3];
	qamax[5]=qmax[5]-0.45*qrange[5];
	qamin[5]=qmin[5]+0.45*qrange[5];

	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0,  -M_PI_2,  0.317    , 0.0     )));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0,   M_PI_2,  0.0    , 0.0     )));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0,  -M_PI_2,  0.45    , 0.0     )));
    	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0,   M_PI_2 ,  0.0    , 0.0     )));
    	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, -M_PI_2,  0.48    , 0.0     )));
    	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, M_PI_2,  0.0    , 0.0     )));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0.0,  0.07    , 0.0     )));
	fk_solver=new KDL::ChainFkSolverPos_recursive (chain);
	ivk_solver=new KDL::ChainIkSolverVel_pinv_red(chain);
	ivk_solver->set_eMh(eMh);

	q.resize(7);
	position_sub=nh.subscribe<sensor_msgs::JointState>("pa10_state", 1, &SimulatedPA10::readJointsCallback, this);
	velocity_pub=nh.advertise<sensor_msgs::JointState>("pa10_command",1);
}

void SimulatedPA10::readJointsCallback(const sensor_msgs::JointState::ConstPtr& state) {
   if (state->position.size()==7)
	for (int i=0; i<7; i++) q[i]=state->position[i];
}

int SimulatedPA10::getPosition(vpHomogeneousMatrix &bMe) {
	vpColVector q(7);
	int res=getJointValues(q);
	if (res) bMe=directKinematics(q);
	
	return res;
}

int SimulatedPA10::setJointValues(vpColVector q) {
	sensor_msgs::JointState js;
	js.name.push_back(std::string("q1"));
  	js.position.push_back(q[0]);
	js.name.push_back(std::string("q2"));
  	js.position.push_back(q[1]);
	js.name.push_back(std::string("q3"));
  	js.position.push_back(q[2]);
	js.name.push_back(std::string("q4"));
  	js.position.push_back(q[3]);
	js.name.push_back(std::string("q5"));
  	js.position.push_back(q[4]);
	js.name.push_back(std::string("q6"));
  	js.position.push_back(q[5]);
	js.name.push_back(std::string("q7"));
  	js.position.push_back(q[6]);
	
	velocity_pub.publish(js);
}

int SimulatedPA10::setJointVelocity(vpColVector qdot) {
	sensor_msgs::JointState js;
	js.name.push_back(std::string("q1"));
  	js.velocity.push_back(qdot[0]);
	js.name.push_back(std::string("q2"));
  	js.velocity.push_back(qdot[1]);
	js.name.push_back(std::string("q3"));
  	js.velocity.push_back(qdot[2]);
	js.name.push_back(std::string("q4"));
  	js.velocity.push_back(qdot[3]);
	js.name.push_back(std::string("q5"));
  	js.velocity.push_back(qdot[4]);
	js.name.push_back(std::string("q6"));
  	js.velocity.push_back(qdot[5]);
	js.name.push_back(std::string("q7"));
  	js.velocity.push_back(qdot[6]);
	
	velocity_pub.publish(js);
	return true;
}

int SimulatedPA10::setCartesianVelocity(vpColVector xdot) {
	KDL::JntArray kq(chain.getNrOfJoints());
	for (int i=0; i<7; i++) kq(i)=q[i];
	//std::cerr << "IK, q is " << q.t() << std::endl;

	//Get current eef position, given q
//	vpHomogeneousMatrix bMe;
//	bMe=directKinematics(q);
	//Transform xdot, given in eef coordinates, to base
//	vpVelocityTwistMatrix bWe(bMe);
//	vpColVector xdot_base=bWe*xdot;

	KDL::Twist v;
	for (int i=0; i<6;i++) v(i)=xdot[i];
	//std::cerr << "IK, xdot is " << xdot.t() << std::endl;

	KDL::JntArray kqdot(chain.getNrOfJoints());
	ivk_solver->CartToJnt(kq,v,kqdot);

	vpColVector qdot(7);	
	for (int i=0; i<7; i++) qdot[i]=kqdot(i);
	//std::cerr << "IK, qdot is " << qdot.t() << std::endl;

	setJointVelocity(qdot);

	return true;
}



vpHomogeneousMatrix SimulatedPA10::directKinematics(vpColVector q) {
	KDL::JntArray kq(chain.getNrOfJoints());

	for (int i=0; i<7; i++) kq(i)=q[i];

	KDL::Frame fkFrame;
	fk_solver->JntToCart(kq,fkFrame);
	vpHomogeneousMatrix bMe;
	for (int i=0; i<4; i++)
	  for (int j=0; j<4; j++) {
		bMe[i][j]=fkFrame(i,j);
	  }

	return bMe;
}


SimulatedPA10::~SimulatedPA10()
{
	delete fk_solver;
	delete ivk_solver;
}


