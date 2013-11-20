#include <mar_robot_arm5e/ARM5Solvers.h>

namespace KDL {

ChainIkSolverVel_pinv_red::ChainIkSolverVel_pinv_red(const Chain& _chain, double _eps, int _maxiter):
        				chain(_chain),
        				jnt2jac(chain),
        				jac(chain.getNrOfJoints()),
        				svd(jac),
        				U(6,JntArray(chain.getNrOfJoints())),
        				S(chain.getNrOfJoints()),
        				V(chain.getNrOfJoints(),JntArray(chain.getNrOfJoints())),
        				tmp(chain.getNrOfJoints()),
        				eps(_eps),
        				maxiter(_maxiter)
{
	fksolver=new ChainFkSolverPos_recursive(chain);
	useBaseJacobian=false;
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
	jnt2jac.JntToJac(q_in,jac);
	//Compute current end-effector frame
	Frame EF;
	fksolver->JntToCart(q_in,EF);

	//Convert HF rotation to visp matrix
	vpHomogeneousMatrix wMe,wMh;
	for (unsigned int i=0; i<4; i++)
		for (unsigned int j=0; j<4; j++) {
			wMe[i][j]=EF(i,j);
		}
	wMh=wMe*eMh;
	vpRotationMatrix wRe, wRh;
	wMe.extract(wRe);
	wMh.extract(wRh);

	//Convert base Jacobian to Visp (J)
	vpMatrix Jb(jac.rows(),jac.columns());
	for (unsigned int r=0; r<jac.rows(); r++)
		for (unsigned int c=0; c<jac.columns(); c++)
			Jb[r][c]=jac(r,c);

	//Compute end-effector jacobian
	vpMatrix J(jac.rows(),jac.columns());
	if (useBaseJacobian) {
		//Use it expressed in the base frame. Assume inputs are velocities of the eef given in the DH base frame
		J=Jb;
	} else {
		//Or expressed on the end-effector. Assume inputs are velocities of the hf given in the current hf
		J=hWe*vpVelocityTwistMatrix(vpTranslationVector(0,0,0),wRe.inverse())*Jb;
	}

	//Remove grasp-redundant DOF's (Jr), and compute the pseudoinverses
	vpMatrix Jr=J;
	for (unsigned int r=3; r<6; r++) {
		for (unsigned int c=0; c<jac.columns(); c++) {
			Jr[r][c]=0;
		}
	}

	vpMatrix Jiv=J.pseudoInverse(eps);
	vpMatrix Jriv=Jr.pseudoInverse(eps);


	//Secondary task: keep hand-to-grasp frame rotation error inside a range
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
	//std::cout << "hRg: " << std::endl << hRg << std::endl;
	//std::cout << "vg: " << vg.t() << std::endl;
	//Compute velocity for desired joint values
	//vpColVector sv(jac.columns());
	//sv=0;
	//sv[4]=0-q_in(4);
	//sv[5]=M_PI_4-q_in(5);
	//sv[6]=M_PI_4-q_in(6);
	//sv[7]=M_PI_2-q_in(7);

	//Secondary task: preferred joint values
	vpColVector sv(jac.columns());
	sv=0;
	if (jac.columns()>5) {
		//The auv dofs are also included
		sv[4]=0-q_in(4);
		sv[5]=M_PI_4-q_in(5);
		sv[6]=M_PI_4-q_in(6);
		sv[7]=M_PI_2-q_in(7);
	} else {
		sv[0]=0;//-q_in(4);
		sv[1]=M_PI_4-q_in(5);
		sv[2]=M_PI_4-q_in(6);
		sv[3]=M_PI_2-q_in(7);
	}


	vpColVector vh(6), qdot(jac.columns());
	for (unsigned int i=0; i<6; i++) {
		vh[i]=v_in[i];
	}

	//Get qdot from desired cartesian velocity
	vpMatrix I(jac.columns(),jac.columns());
	I.setIdentity();
	qdot=Jriv*vh+(I-Jriv*Jr)*sv; /* *(0.8*Jiv*vg+0.5*sv);*/

	for (unsigned int i=0; i<jac.columns(); i++) {
		qdot_out(i)=qdot[i];
	}

	return 1;
}

}

