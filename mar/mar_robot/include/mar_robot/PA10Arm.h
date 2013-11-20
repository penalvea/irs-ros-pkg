#ifndef SIMULATEDPA10_H
#define SIMULATEDPA10_H

#include <string>
#include <vector>

#include "Arm.h"

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

//KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

//ViSP
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>

using namespace std;

namespace KDL
{

    class ChainIkSolverVel_pinv_red : public ChainIkSolverVel
    {
    public:

        ChainIkSolverVel_pinv_red(const Chain& chain, /* const Frame &f,*/ double eps=0.00001,int maxiter=150);
        ~ChainIkSolverVel_pinv_red();

        virtual int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);

        virtual int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out){return -1;};
	
	/** Sets the hand frame */
        void set_eMh(vpHomogeneousMatrix &eMh) 
		{this->eMh=eMh; eWh.buildFrom(eMh); hWe.buildFrom(eMh.inverse()); std::cerr << "eMh set to: " << std::endl << eMh << std::endl;}

    private:
        const Chain chain;
        ChainJntToJacSolver jnt2jac;
        Jacobian jac;
        SVD_HH svd;
        std::vector<JntArray> U;
        JntArray S;
        std::vector<JntArray> V;
        JntArray tmp;
	ChainFkSolverPos_recursive *fksolver;
	/*Frame GF;*/
        double eps;
        int maxiter;

	vpHomogeneousMatrix eMh; 	///< Hand frame
	vpVelocityTwistMatrix eWh, hWe;	///< Twist transformation matrix hand-eef frame
    };

    ChainIkSolverVel_pinv_red::ChainIkSolverVel_pinv_red(const Chain& _chain,  /* const Frame &f,*/ double _eps,int _maxiter):
        chain(_chain),
        jnt2jac(chain),
        jac(chain.getNrOfJoints()),
        svd(jac),
        U(6,JntArray(chain.getNrOfJoints())),
        S(chain.getNrOfJoints()),
        V(chain.getNrOfJoints(),JntArray(chain.getNrOfJoints())),
        tmp(chain.getNrOfJoints()),
	/*GF(f),*/
        eps(_eps),
        maxiter(_maxiter)
    {
	fksolver=new ChainFkSolverPos_recursive(chain);
    }

    ChainIkSolverVel_pinv_red::~ChainIkSolverVel_pinv_red()
    {
	delete fksolver;
    }


    int ChainIkSolverVel_pinv_red::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
    {
        //Let the ChainJntToJacSolver calculate the jacobian "jac" for
        //the current joint positions "q_in" 
	//CAUTION: JntToJac computes the end-effector Jacobian, but expressed on the BASE FRAME
	//std::cerr << "jntTojac" << std::endl;
        jnt2jac.JntToJac(q_in,jac);
	//Compute current end-effector frame
	Frame EF;
	fksolver->JntToCart(q_in,EF);

	//Convert EF and GF to visp matrix
	vpHomogeneousMatrix wMe, wMh;
	//vpHomogeneousMatrix wMg;
	for (unsigned int i=0; i<4; i++) 
		for (unsigned int j=0; j<4; j++) {
			wMe[i][j]=EF(i,j);
	//		wMg[i][j]=GF(i,j);
		}
	wMh=wMe*eMh;
	vpRotationMatrix wRe;
	wMe.extract(wRe);
	//std::cout << "wRh: " << std::endl << wRh << std::endl;
	//vpRotationMatrix wRg;
	//wMg.extract(wRg);
	//std::cout << "wRg: " << std::endl << wRg << std::endl;

	//Convert base Jacobian to Visp (J)
	vpMatrix Jb(jac.rows(),jac.columns());
	for (unsigned int r=0; r<jac.rows(); r++)
		for (unsigned int c=0; c<jac.columns(); c++) 
			Jb[r][c]=jac(r,c);

	//Compute end-effector jacobian expressed on the hand frame
	vpMatrix J(jac.rows(),jac.columns());
	J=hWe*vpVelocityTwistMatrix(vpTranslationVector(0,0,0),wRe.inverse())*Jb;

	//remove rotation DOF's (Jr), and compute de pseudoinverses
	vpMatrix Jr=J;
	for (unsigned int r=4; r<5; r++) {
		for (unsigned int c=0; c<jac.columns(); c++) {
			Jr[r][c]=0;
		}
	}

	vpMatrix Jiv=J.pseudoInverse(eps);
	vpMatrix Jriv=Jr.pseudoInverse(eps);


	//Compute secondary task: keep hand-to-grasp frame rotation error inside a range	

	
	//Compute RPY angles between H and G
	//vpRotationMatrix hRg=wRh.inverse()*wRg;
	//vpRxyzVector hRPYg(hRg);
	//Compute grasp frame velocity towards hand frame	
	//vpPoseVector vg(vpTranslationVector(0,0,0),hRg);;
	//vg[5]=0;
	//vpColVector vg(6);
	//vg=0;
	//vg[3]=hRPYg[0];
	//vg[4]=hRPYg[1];
	//vg[5]=hRPYg[2];
	vpColVector hz(3), wz(3);
	hz[0]=wMh[0][2];
	hz[1]=wMh[1][2];
	hz[2]=wMh[2][2];
	wz=0; wz[2]=1;
	double angle=acos(vpColVector::dotProd(hz,wz)/(hz.euclideanNorm()*wz.euclideanNorm()));
	std::cerr << "angle: " << M_PI_2-angle << std::endl;

	
	//std::cout << "hRg: " << std::endl << hRg << std::endl;
	//std::cout << "vg: " << vg.t() << std::endl;
	//Compute velocity for desired joint values
	vpColVector sv(jac.columns());
	sv[0]=0-q_in(0);
	sv[1]=M_PI_4-q_in(1);
	sv[2]=0-q_in(2);
	sv[3]=-M_PI_2-q_in(3);
	sv[4]=0-q_in(4);
	sv[5]=-M_PI_4/2-q_in(5);
	sv[6]=0-q_in(6);
	//std::cout << "sv: " << sv.t() << std::endl;
	

	vpColVector v(6), ve(6), qdot(jac.columns());
	for (unsigned int i=0; i<6; i++) {
		v[i]=v_in[i];
	}
	//vpVelocityTwistMatrix hWw(vpTranslationVector(0,0,0),wRh.inverse());
	//ve=hWw*v;
	vpMatrix I(jac.columns(),jac.columns());
	I.setIdentity();
	std::cerr << "svnorm: " << sv.euclideanNorm() << std::endl;
	qdot=Jriv*v  +(I-Jriv*Jr)*( /* 0.2*Jiv*vg+ */ 0.01*sv);

	for (int i=0; i<jac.columns(); i++) {
		qdot_out(i)=qdot[i];
	}

        return 1;
    }
}


/**
Interface for a PA10 arm
	@author Mario Prats <mprats@icc.uji.es>
*/
class SimulatedPA10: public Arm
{
private: 
	ros::Subscriber position_sub;
	ros::Publisher velocity_pub;

	KDL::Chain chain;		//KDL Chain
	KDL::ChainFkSolverPos_recursive *fk_solver;	//KDL forward kinematics solver
	KDL::ChainIkSolverVel_pinv_red  *ivk_solver;	//KDL inverse velocity solver

	vpColVector q;
	vpHomogeneousMatrix eMh; 	///< Hand frame
	vpVelocityTwistMatrix eWh;	///< Twist transformation matrix hand-eef frame

public:
    /** The constructor */
    SimulatedPA10(ros::NodeHandle &nh);

    /** Gets the current end-effector pose w.r.t robot kinematic base (units in meters) */
    virtual int getPosition(vpHomogeneousMatrix &bMe);

    /** Gets the current joint values */
    virtual int getJointValues(vpColVector &q) {q=this->q; return 1;}
    void readJointsCallback(const sensor_msgs::JointState::ConstPtr& state);


    /** Sets the desired joint positions */
    virtual int setJointValues(vpColVector dq);
    /** Sets the desired joint velocities */
    virtual int setJointVelocity(vpColVector qdot);
    /** Sets the desired cartesian velocities in the hand frame. Units in meters/s and radians/s */
    virtual int setCartesianVelocity(vpColVector qdot);

    /** Sets the hand frame */
    void set_eMh(vpHomogeneousMatrix &eMh) 
	{this->eMh=eMh; eWh.buildFrom(eMh); if (ivk_solver!=NULL) ivk_solver->set_eMh(eMh);}

    ~SimulatedPA10();

    /** Computes the forward kinematics of the arm */
    vpHomogeneousMatrix directKinematics(vpColVector q);



private:
    vpColVector qmax, qmin, qrange, qamax, qamin;   ///< Allowed joint range
};

#endif
