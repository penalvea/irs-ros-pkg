#include <mar_robot_arm5e/ARM5Arm.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <ros/ros.h>
#include <arm5_controller/setZero.h>

#include <std_msgs/Float32MultiArray.h>

void ARM5Arm::initKinematicSolvers() {
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
	//chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 5.0*M_PI/180     )));//7.98
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 7.98*M_PI/180     )));//7.98
	//chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 ,  M_PI_2,  0.0    , 127.02*M_PI/180     )));//127.02
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 ,  M_PI_2,  0.0    , 113.0*M_PI/180     )));//127.02
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.49938+0.45    , 0.0     )));

	/*chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0911,  M_PI_2,  0.0    , 0.0     )));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.4650, 0.0 ,  0.0    , 0.1119     )));
	
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.5094,  M_PI_2,  0.0    , 3.5158     )));
	chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.0525    , 0.0     )));*/



	fksolver = new KDL::ChainFkSolverPos_recursive(chain);

	qmax.resize(5);
	qmin.resize(5);
	qrange.resize(5);
	qamax.resize(5);
	qamin.resize(5);
	q.resize(5);

	qmax[0]=30*M_PI/180;
	qmax[1]=90*M_PI/180;
	qmax[2]=120*M_PI/180;
	qmax[3]=180*M_PI/180;
	qmax[4]=45*M_PI/180;

	qmin[0]=-90*M_PI/180;
	qmin[1]=0*M_PI/180;
	qmin[2]=0*M_PI/180;
	qmin[3]=-180*M_PI/180;
	qmin[4]=0*M_PI/180;

	qrange=qmax-qmin;
	qamax=qmax-0.1*qrange;
	qamin=qmin+0.1*qrange;

	is_initialized=false;


	KDL::JntArray opt_pos(4);
	opt_pos(0)=0.0;
	opt_pos(1)=0.7;
	opt_pos(2)=0.7;
	opt_pos(3)=0.0;

	KDL::JntArray weight(4);
	weight(0)=0;
	weight(1)=0;
	weight(2)=0;
	weight(3)=0;

	ivk_solver=new KDL::ChainIkSolverVel_pinv_red(chain);

	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransX)));
	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransY)));
	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ)));
	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0,  0,  1.08    , M_PI     )));
	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 5.0*M_PI/180     )));
	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 /* gripper pos. changed */,  M_PI_2,  0.0    , 115*M_PI/180     )));
	auvarm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.49938    , 0.0     )));
	auvarm_ivk_solver=new KDL::ChainIkSolverVel_pinv_red(auvarm_chain);


	armhotstab_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.08052,  M_PI_2,  0.0    , 0.0     )));
	armhotstab_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.44278, 0.0 ,  0.0    , 5.0*M_PI/180     )));
	armhotstab_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( -0.083 /* gripper pos. changed */,  M_PI_2,  0.0    , 115*M_PI/180     )));
	armhotstab_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), KDL::Frame().DH( 0.0, 0,  0.74938    , 0.0     )));
  armhotstab_ivk_solver=new KDL::ChainIkSolverVel_pinv_red(armhotstab_chain);
}

ARM5Arm::ARM5Arm() : current(0)
{
	initKinematicSolvers();
}

ARM5Arm::ARM5Arm(ros::NodeHandle &nh) :  nh_(nh), current(0)
{
	initKinematicSolvers();
	position_sub=nh.subscribe<sensor_msgs::JointState>(DEFAULT_STATE_TOPIC, 1, &ARM5Arm::readJointsCallback, this);
	velocity_pub=nh.advertise<sensor_msgs::JointState>(DEFAULT_CONTROL_TOPIC,1);
}

ARM5Arm::ARM5Arm(ros::NodeHandle &nh, std::string state_topic, std::string control_topic) :  nh_(nh), current(0)
{

	initKinematicSolvers();

	position_sub=nh.subscribe<sensor_msgs::JointState>(state_topic, 1, &ARM5Arm::readJointsCallback, this);

	velocity_pub=nh.advertise<sensor_msgs::JointState>(control_topic,1);


}

void ARM5Arm::readJointsCallback(const sensor_msgs::JointState::ConstPtr& state) {
	for (int i=0; i<5; i++) q[i]=state->position[i];
	current=state->effort[0];
	is_initialized=true;
	vpHomogeneousMatrix bMe=directKinematics(q);
}


int ARM5Arm::init(bool q1, bool q2, bool q3, bool q4, bool q5) {
	if (!isInit()) return forcedInit(q1,q2,q3,q4,q5);
	else return is_initialized;
}

int ARM5Arm::forcedInit(bool q1, bool q2, bool q3, bool q4, bool q5) {
	ROS_INFO("Initializing ARM5E joints...");
	InitCSIP init_csip(nh_,q1,q2,q3,q4,q5);

	bool offsetsDone=false;

	ros::Rate r(20);
	while (ros::ok() && !offsetsDone) {
		ros::spinOnce();
		init_csip.autoInitVel();
		if (init_csip.activeAxis>=5 && !offsetsDone) {
			ROS_INFO("All axis initialized. Sending zero offsets to the arm...");
			init_csip.callSetZeroService();
			offsetsDone=true;
		}
		r.sleep();
	}

	return offsetsDone;
}

int ARM5Arm::getPosition(vpHomogeneousMatrix &bMe) {
	vpColVector q(5);

	int res=getJointValues(q);
	if (res) bMe=directKinematics(q);

	return res;
}


int ARM5Arm::getJointValues(vpColVector &q) {
	q=this->q;

	return true;
}

int ARM5Arm::setJointValues(vpColVector &q) {
	return setJointValues(q, false);
}

int ARM5Arm::setJointValues(double dq[5], bool blocking, double tol) {
	vpColVector qv(5);
	memcpy(qv.data, dq, 5*sizeof(double));
	return setJointValues(qv, blocking, tol);
}

int ARM5Arm::setJointValues(vpColVector &q, bool blocking, double tol) {
	do {
		sensor_msgs::JointState js;
		js.name.push_back(std::string("Slew"));
		js.position.push_back(q[0]);
		js.name.push_back(std::string("Shoulder"));
		js.position.push_back(q[1]);
		js.name.push_back(std::string("Elbow"));
		js.position.push_back(q[2]);
		js.name.push_back(std::string("JawRotate"));
		js.position.push_back(q[3]);
		js.name.push_back(std::string("JawOpening"));
		js.position.push_back(q[4]);

		js.header.stamp=ros::Time::now();
		velocity_pub.publish(js);
		if (blocking) ros::spinOnce();
               // std::cout<<(this->q-q).euclideanNorm()<<std::endl;
	} while(blocking && ros::ok() && (this->q-q).euclideanNorm()>tol);
	return ros::ok();
}

int ARM5Arm::setJointVelocity(vpColVector qdot) {
	vpColVector qsat(5);
	qsat=qdot;
	//for (int i=0; i<5; i++)
	//      if (qsat[i]>0.2) qsat[i]=0.2;
	//      else if (qsat[i]<-0.2) qsat[i]=-0.2;
	
	if(qsat[0]<0.06 && qsat[0]>0.01)
		qsat[0]=0.06;
	if(qsat[0]>-0.06 && qsat[0]<-0.01)
		qsat[0]=-0.06;
	if(qsat[1]<0.06 && qsat[1]>0.01)
		qsat[1]=0.06;
	if(qsat[1]>-0.06 && qsat[1]<-0.01)
		qsat[1]=-0.06;
	if(qsat[2]<0.06 && qsat[2]>0.01)
		qsat[2]=0.06;
	if(qsat[2]>-0.06 && qsat[2]<-0.01)
		qsat[2]=-0.06;
	if(qsat[3]<0.2 && qsat[3]>0.01)
		qsat[3]=0.2;
	if(qsat[3]>-0.2 && qsat[3]<-0.01)
		qsat[3]=-0.2;
	if(qsat[4]<0.1 && qsat[4]>0.01)
		qsat[4]=0.1;
	if(qsat[4]>-0.1 && qsat[4]<-0.01)
		qsat[4]=-0.1;

	sensor_msgs::JointState js;
	js.name.push_back(std::string("Slew"));
	js.velocity.push_back(qsat[0]);
	js.name.push_back(std::string("Shoulder"));
	js.velocity.push_back(qsat[1]);
	js.name.push_back(std::string("Elbow"));
	js.velocity.push_back(qsat[2]);
	js.name.push_back(std::string("JawRotate"));
	js.velocity.push_back(qsat[3]);
	js.name.push_back(std::string("JawOpening"));
	js.velocity.push_back(qsat[4]);

	js.header.stamp=ros::Time::now();
	velocity_pub.publish(js);
	return true;
}

bool ARM5Arm::checkJointLimits(std::vector<int> &inlimit) {
	//Check joint limits:
	vpColVector q(4);
	getJointValues(q);
	inlimit.clear();

	if (q[0]<=qmin[0] || q[0]>=qmax[0]) {
		inlimit.push_back(0);
		std::cerr << "ARM5Arm::checkJointLimits ERROR: Joint 0 is in joint limit" << std::endl;
	} else if (q[1]<=qmin[1] || q[1]>=qmax[1]) {
		inlimit.push_back(1);
		std::cerr << "ARM5Arm::checkJointLimits ERROR: Joint 1 is in joint limit" << std::endl;
	} else if (q[2]<=qmin[2] || q[2]>=qmax[2]) {
		inlimit.push_back(2);
		std::cerr << "ARM5Arm::checkJointLimits ERROR: Joint 2 is in joint limit" << std::endl;
	} else if (q[3]<=qmin[3] || q[3]>=qmax[3]) {
		inlimit.push_back(3);
		std::cerr << "ARM5Arm::checkJointLimits ERROR: Joint 3 is in joint limit" << std::endl;
	}
	return (inlimit.size()>0);
}

int ARM5Arm::setCartesianVelocity(vpColVector xdot) {
	KDL::JntArray kq(chain.getNrOfJoints());
	for (int i=0; i<4; i++) kq(i)=q[i];
	//	std::cerr << "IK, q is " << q.t() << std::endl;

	//Get current eef position, given q
	//	vpHomogeneousMatrix bMe;
	//	bMe=directKinematics(q);
	//Transform xdot, given in eef coordinates, to base
	//	vpVelocityTwistMatrix bWe(bMe);
	//	vpColVector xdot_base=bWe*xdot;

	//Check discontinuities in xdot

	static vpColVector xdot_previous(6);
	vpColVector xdot_filtered(6);
	xdot_filtered=xdot;
	if ((xdot-xdot_previous).euclideanNorm()>DISC_EPSILON) {
		std::cerr << "Detected cartesian velocity discontinuity. Norm of the difference: " << (xdot-xdot_previous).euclideanNorm() << std::endl;
		xdot_filtered=xdot_previous+(xdot-xdot_previous)*(DISC_EPSILON/(xdot-xdot_previous).euclideanNorm());
		std::cerr << "New interpolated velocity will be: " << xdot_filtered.t() << std::endl;
	}
	xdot_previous=xdot_filtered;

	KDL::Twist v;
	for (int i=0; i<6;i++) v(i)=xdot_filtered[i];
	std::cerr << "IK, xdot is " << xdot.t() << std::endl;

	KDL::JntArray kqdot(chain.getNrOfJoints());
	ivk_solver->CartToJnt(kq,v,kqdot);

	vpColVector qdot(5);	
	for (int i=0; i<4; i++) qdot[i]=kqdot(i);
	qdot[4]=0;
	std::cerr << "IK, qdot is " << qdot.t() << std::endl;
	qdot[3]=0;
	setJointVelocity(qdot);

	return true;
}

/** Compute the IK of the arm for reaching a given frame wMe (=bMe)*/
vpColVector ARM5Arm::armIK(vpHomogeneousMatrix &wMe){
	vpColVector maxJointLimits(4), minJointLimits(4);
	maxJointLimits[0]=30*M_PI/180;
	maxJointLimits[1]=90*M_PI/180;
	maxJointLimits[2]=145*M_PI/180;
	maxJointLimits[3]=360*M_PI/180;

	minJointLimits[0]=-90*M_PI/180;
	minJointLimits[1]=0*M_PI/180;
	minJointLimits[2]=0*M_PI/180;
	minJointLimits[3]=-360*M_PI/180;
	return armIK(wMe, maxJointLimits, minJointLimits);

}
vpColVector ARM5Arm::armIK(vpHomogeneousMatrix &wMe,vpColVector maxJointLimits, vpColVector minJointLimits) {
vpColVector initial_joints(4);
initial_joints[0]=0;
initial_joints[1]=M_PI_4;
initial_joints[2]=M_PI_4;
initial_joints[3]=0;
return armIK(wMe, maxJointLimits, minJointLimits, initial_joints);


}
vpColVector ARM5Arm::armIK(vpHomogeneousMatrix &wMe, vpColVector maxJointLimits, vpColVector minJointLimits, vpColVector initial_joints){
	KDL::JntArray q(chain.getNrOfJoints());
	KDL::JntArray q_init(chain.getNrOfJoints());
	KDL::JntArray qmin(chain.getNrOfJoints());
	KDL::JntArray qmax(chain.getNrOfJoints());

	//Joint limits
	qmax(0)=maxJointLimits[0];
	qmax(1)=maxJointLimits[1];
	qmax(2)=maxJointLimits[2];
	qmax(3)=maxJointLimits[3];

	qmin(0)=minJointLimits[0];
	qmin(1)=minJointLimits[1];
	qmin(2)=minJointLimits[2];
	qmin(3)=minJointLimits[3];

	//Set destination frame
	vpTranslationVector wTe;
	vpRotationMatrix wRe;
	wMe.extract(wTe);
	wMe.extract(wRe);

	KDL::Rotation rot;
	for (int i=0; i<9; i++) rot.data[i]=wRe.data[i];
	KDL::Vector tvec;	
	for (int i=0; i<3; i++) tvec.data[i]=wTe.data[i];
	KDL::Frame F_dest(rot,tvec);

	/*std::cout << "F_dest (wMe): " << std::endl;
	for (int i=0; i<4; i++) {
		for (int j=0; j<4; j++) {
			std::cout << F_dest(i,j) << " ";
		}
		std::cout << std::endl;
	} */


	//Kinematic solvers
	KDL::ChainFkSolverPos_recursive fksolver(chain);//Forward position solver
	KDL::ChainIkSolverVel_pinv_red iksolverv(chain);//Custom Inverse velocity solver (grasp redundancy)
	iksolverv.setBaseJacobian(true);
	//KDL::ChainIkSolverPos_NR iksolver(chain,fksolver,iksolverv,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
	KDL::ChainIkSolverPos_NR_JL iksolver(chain, qmin, qmax, fksolver,iksolverv,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	//Initial guess
  q_init(0)=initial_joints[0];
  q_init(1)=initial_joints[1];
  q_init(2)=initial_joints[2];
  q_init(3)=initial_joints[3];

	iksolver.CartToJnt(q_init,F_dest,q);

	//return result
	vpColVector qv(chain.getNrOfJoints());
	for (unsigned int i=0; i<chain.getNrOfJoints(); i++) {
		qv[i]=q(i);
	}
	return qv;
}

/** Compute the IK of the vehicle-arm for reaching a given frame wMe */
vpColVector ARM5Arm::vehicleArmIK(vpHomogeneousMatrix &wMe){
        vpColVector maxJointLimits(8), minJointLimits(8);
        maxJointLimits[0]=100;
        maxJointLimits[1]=100;
        maxJointLimits[2]=100;
        maxJointLimits[3]=360*M_PI/180;
        maxJointLimits[4]=30*M_PI/180;
        maxJointLimits[5]=90*M_PI/180;
        maxJointLimits[6]=145*M_PI/180;
        maxJointLimits[7]=360*M_PI/180;

        minJointLimits[0]=-100;
        minJointLimits[1]=-100;
        minJointLimits[2]=-100;
        minJointLimits[3]=-360*M_PI/180;
        maxJointLimits[4]=30*M_PI/180;
        maxJointLimits[5]=90*M_PI/180;
        maxJointLimits[6]=145*M_PI/180;
        maxJointLimits[7]=360*M_PI/180;
        return vehicleArmIK(wMe, maxJointLimits, minJointLimits);
}
vpColVector ARM5Arm::vehicleArmIK(vpHomogeneousMatrix &wMe,vpColVector maxJointLimits, vpColVector minJointLimits) {
	KDL::JntArray q(auvarm_chain.getNrOfJoints());
	KDL::JntArray q_init(auvarm_chain.getNrOfJoints());
	KDL::JntArray qmin(auvarm_chain.getNrOfJoints());
	KDL::JntArray qmax(auvarm_chain.getNrOfJoints());

	//Joint limits
	qmax(0)=maxJointLimits[0];
        qmax(1)=maxJointLimits[1];
        qmax(2)=maxJointLimits[2];
        qmax(3)=maxJointLimits[3];
        qmax(4)=maxJointLimits[4];
        qmax(5)=maxJointLimits[5];
        qmax(6)=maxJointLimits[6];
        qmax(7)=maxJointLimits[7];

        qmin(0)=minJointLimits[0];
        qmin(1)=minJointLimits[1];
        qmin(2)=minJointLimits[2];
        qmin(3)=minJointLimits[3];
        qmin(4)=minJointLimits[4];
        qmin(5)=minJointLimits[5];
        qmin(6)=minJointLimits[6];
        qmin(7)=minJointLimits[7];
        qmin(0)=minJointLimits[0];

	//Set destination frame
	vpTranslationVector wTe;
	vpRotationMatrix wRe;
	wMe.extract(wTe);
	wMe.extract(wRe);
	KDL::Rotation rot;
	for (int i=0; i<9; i++) rot.data[i]=wRe.data[i];
	KDL::Vector tvec;	
	for (int i=0; i<3; i++) tvec.data[i]=wTe.data[i];
	KDL::Frame F_dest(rot,tvec);

	/*std::cout << "F_dest: " << std::endl;
	for (int i=0; i<4; i++) {
		for (int j=0; j<4; j++) {
			std::cout << F_dest(i,j) << " ";
		}
		std::cout << std::endl;
	}*/

	//Kinematic solvers
	KDL::ChainFkSolverPos_recursive fksolver(auvarm_chain);//Forward position solver
	KDL::ChainIkSolverVel_pinv_red iksolverv(auvarm_chain);//Custom Inverse velocity solver (grasp redundancy)
	iksolverv.setBaseJacobian(true);
	//KDL::ChainIkSolverPos_NR iksolver(auvarm_chain, fksolver,iksolverv,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
	KDL::ChainIkSolverPos_NR_JL iksolver(auvarm_chain, qmin, qmax, fksolver,iksolverv,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	//Initial guess
	q_init(0)=0;
	q_init(1)=0;
	q_init(2)=0;
	q_init(3)=0;
	q_init(4)=0;
	q_init(5)=M_PI_4;
	q_init(6)=M_PI_4;
	q_init(7)=0;

	iksolver.CartToJnt(q_init,F_dest,q);

	//return result
	vpColVector qv(auvarm_chain.getNrOfJoints());
	for (unsigned int i=0; i<auvarm_chain.getNrOfJoints(); i++) {
		qv[i]=q(i);
	}
	return qv;
}


/** Compute the IK of the arm-hotStab for reaching a given frame wMe (=bMe) */
vpColVector ARM5Arm::armHotStabIK(vpHomogeneousMatrix &wMe) {
	KDL::JntArray q(armhotstab_chain.getNrOfJoints());
	KDL::JntArray q_init(armhotstab_chain.getNrOfJoints());
	KDL::JntArray qmin(armhotstab_chain.getNrOfJoints());
	KDL::JntArray qmax(armhotstab_chain.getNrOfJoints());

	//Joint limits
	qmax(0)=30*M_PI/180;
	qmax(1)=90*M_PI/180;
	qmax(2)=145*M_PI/180;
	qmax(3)=360*M_PI/180;

	qmin(0)=-90*M_PI/180;
	qmin(1)=0*M_PI/180;
	qmin(2)=0*M_PI/180;
	qmin(3)=-360*M_PI/180;

	//Set destination frame
	vpTranslationVector wTe;
	vpRotationMatrix wRe;
	wMe.extract(wTe);
	wMe.extract(wRe);

	KDL::Rotation rot;
	for (int i=0; i<9; i++) rot.data[i]=wRe.data[i];
	KDL::Vector tvec;
	for (int i=0; i<3; i++) tvec.data[i]=wTe.data[i];
	KDL::Frame F_dest(rot,tvec);

	/*std::cout << "F_dest (wMe): " << std::endl;
		for (int i=0; i<4; i++) {
			for (int j=0; j<4; j++) {
				std::cout << F_dest(i,j) << " ";
			}
			std::cout << std::endl;
		} */


	//Kinematic solvers
	KDL::ChainFkSolverPos_recursive fksolver(armhotstab_chain);//Forward position solver
	KDL::ChainIkSolverVel_pinv_red iksolverv(armhotstab_chain);//Custom Inverse velocity solver (grasp redundancy)
	iksolverv.setBaseJacobian(true);
	KDL::ChainIkSolverPos_NR iksolver(armhotstab_chain,fksolver,iksolverv,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

	//Initial guess
	q_init(0)=0;
	q_init(1)=M_PI_4;
	q_init(2)=M_PI_4;
	q_init(3)=0;

	iksolver.CartToJnt(q_init,F_dest,q);

	//return result
	vpColVector qv(armhotstab_chain.getNrOfJoints());
	for (unsigned int i=0; i<armhotstab_chain.getNrOfJoints(); i++) {
		qv[i]=q(i);
	}
	return qv;
}

/** Compute the most suitable camera-object relative position for reaching a given frame wMg 
 * cMb is the camera-to-armbase calibration
 * oMg is the grasp frame wrt the object frame
 * eMh is the hand frame wrt the eef frame
 * wMg is the grasp frame in world coordinates
 */
vpHomogeneousMatrix ARM5Arm::compute_best_cMo(vpHomogeneousMatrix cMb, vpHomogeneousMatrix oMg, vpHomogeneousMatrix eMh, vpHomogeneousMatrix wMg) {
	vpColVector q;
	vpHomogeneousMatrix wMe=wMg*eMh.inverse();

	q=vehicleArmIK(wMe);

	//std::cout << "wMe is: " << std::endl << wMe << std::endl;
	std::cout << "wMg is: " << std::endl << wMg << std::endl;
	std::cout << "IK: " << q.t() << std::endl;

	KDL::JntArray jq(4);
	for(unsigned int i=4;i<q.getRows();i++)
		jq(i-4)=q[i];

	//std::cerr << "jq: ";
	for (int i=0; i<4; i++) std::cerr << jq(i) << " ";
	std::cerr << std::endl;

	vpHomogeneousMatrix bMe, bMh;
	KDL::Frame cartpos;
	fksolver->JntToCart(jq,cartpos);
	for (int i=0; i<4; i++)
		for (int j=0;j<4;j++)
			bMe[i][j]=cartpos(i,j);
	std::cerr << "bMe is: " << std::endl << bMe << std::endl;
	//std::cerr << "eMh is: " << std::endl << eMh << std::endl;
	bMh=bMe*eMh;
	//std::cerr << "bMh is: " << std::endl << bMh << std::endl;
	vpHomogeneousMatrix cMo;
	cMo=cMb*bMh*oMg.inverse();

	return cMo;	
}


vpHomogeneousMatrix ARM5Arm::directKinematics(vpColVector q) {

	vpHomogeneousMatrix wMe;

	KDL::JntArray jq(q.getRows()-1);
	for(unsigned int i=0;i<q.getRows()-1;i++)
		jq(i)=q[i];

	KDL::Frame cartpos;
	fksolver->JntToCart(jq,cartpos);
	for (int i=0; i<4; i++){
		for (int j=0;j<4;j++){
			wMe[i][j]=cartpos(i,j);
		}
	}

	return wMe;
}


ARM5Arm::~ARM5Arm()
{
	//	delete jsensor;
	//	delete jactuator;
	//	delete fksolver;
}



InitCSIP::InitCSIP(ros::NodeHandle &nh, bool q1, bool q2, bool q3, bool q4, bool q5)
{
  current=0.0;
  //Default values. Do not initialize any axis
  initAxis[0]=q1;
  initAxis[1]=q2;
  initAxis[2]=q3;
  initAxis[3]=q4;
  initAxis[4]=q5;
  for (int i=0; i<5; i++) {
    initAxisDone[i]=false;
    AxisDir[i]=1;
    initAxisThreshold[i]=0.6;
    limitPosition[i]=0;
  }
  initAuto=true;

  //Read from the parameter server
  nh.param("initSlewThreshold", initAxisThreshold[0], initAxisThreshold[0]);
  nh.param("initShoulderThreshold", initAxisThreshold[1], initAxisThreshold[1]);
  nh.param("initElbowThreshold", initAxisThreshold[2], initAxisThreshold[2]);
  nh.param("initWristThreshold", initAxisThreshold[3], initAxisThreshold[3]);
  nh.param("initJawThreshold", initAxisThreshold[4], initAxisThreshold[4]);

  for (int i=0; i<5; i++) {
    ROS_INFO_STREAM("Axis " << i << " threshold: " << initAxisThreshold[i]);
  }

  scale_=2000;
  nh.param("scale", scale_, scale_);
  nh.param("SlewDir", AxisDir[0], AxisDir[0]);
  nh.param("ShoulderDir", AxisDir[1], AxisDir[1]);
  nh.param("ElbowDir", AxisDir[2], AxisDir[2]);
  nh.param("WristDir", AxisDir[3], AxisDir[3]);
  nh.param("JawDir", AxisDir[4], AxisDir[4]);


  activeAxis=0;
  //while (activeAxis<5 && !initAxis[activeAxis]) activeAxis++;
  //if (activeAxis<5)
  ROS_INFO_STREAM("Initializing joint " << activeAxis << ". Move the joint to the limit...");

  //publish joint velocity
  vel_pub_ = nh.advertise<sensor_msgs::JointState>("/arm5e/command_ticks",1);
  //subscribe to obtain joint feedback
  js_sub_ = nh.subscribe<sensor_msgs::JointState>("/arm5e/joint_state_rticks", 1, &InitCSIP::stateCallback, this);
  //service
  setZeroClient = nh.serviceClient<arm5_controller::setZero>("setZero");
}

void InitCSIP::stateCallback(const sensor_msgs::JointState::ConstPtr& state)
{
	if (state->effort.size()>0) {
		current=state->effort[0];
		for (int i=0; i<5; i++) position[i]=state->position[i];
	}

	if (activeAxis<5) {
		if (initAxis[activeAxis]) {
			//std::cerr << "Current is: " << current << std::endl;
			//send velocity to activeAxis (done by joystick callback), check current
			if (current>initAxisThreshold[activeAxis] && !initAxisDone[activeAxis]) {
				//limit reached
				limitPosition[activeAxis]=position[activeAxis];
				initAxisDone[activeAxis]=true;
				ROS_INFO_STREAM("Joint " << activeAxis << " limit at position " << limitPosition[activeAxis]);
				ROS_INFO_STREAM("Waiting 5 sec for the next");
				limitDetected = ros::Time::now();
			} else if (initAxisDone[activeAxis]) {
				//wait 5 sec.
				if ((ros::Time::now()-limitDetected).toSec() > 5) {
					//Move to the next joint
					//while (activeAxis<5 && (!initAxis[activeAxis] || initAxisDone[activeAxis])) activeAxis++;
					activeAxis++;
					if (activeAxis<5) ROS_INFO_STREAM("Initializing joint " << activeAxis << ". Move the joint to the limit...");
				}
			}
		} else {
			//Do not initialize axis at the limit. Assume it has manually brought to zero and set offsets according to a previous calibration
			limitPosition[activeAxis]=position[activeAxis]-ARM5EZeroAbsTicks[activeAxis];
			activeAxis++;
			if (activeAxis<5) ROS_INFO_STREAM("Initializing joint " << activeAxis << ". Move the joint to the limit...");
		}
	}
}

void InitCSIP::autoInitVel()
{
	sensor_msgs::JointState js;
	js.name.push_back(std::string("Slew"));
	js.name.push_back(std::string("Shoulder"));
	js.name.push_back(std::string("Elbow"));
	js.name.push_back(std::string("JawRotate"));
	js.name.push_back(std::string("JawOpening"));

	js.velocity.resize(5);
	for (int i=0; i<5; i++) js.velocity[i]=0;
	if (activeAxis<5 && initAxis[activeAxis] && !initAxisDone[activeAxis]) {
		js.velocity[activeAxis]=AxisDir[4]*scale_;
	}
	js.header.stamp=ros::Time::now();
	vel_pub_.publish(js);
}


int InitCSIP::callSetZeroService() {
	arm5_controller::setZero srv;

	for (int i=0; i<5; i++)
		srv.request.zeroOffsets.push_back(limitPosition[i]);

	if (setZeroClient.call(srv))
	{
		ROS_INFO("Zero offsets have been set: %d %d %d %d %d\n", limitPosition[0],limitPosition[1],limitPosition[2],limitPosition[3],limitPosition[4]);
	}
	else
	{
		ROS_ERROR("Failed to call service setZero");
		return 0;
	}
	return 1;
}

