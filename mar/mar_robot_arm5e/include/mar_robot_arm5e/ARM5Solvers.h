/*
 * ARM5Solvers.h
 *
 *  Created on: 24/05/2012
 *      Author: mprats
 */

#ifndef ARM5SOLVERS_H_
#define ARM5SOLVERS_H_

//KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>

namespace KDL
{
class ChainIkSolverVel_pinv_red : public ChainIkSolverVel
{
public:

	ChainIkSolverVel_pinv_red(const Chain& chain, double eps=0.00001,int maxiter=150);

	~ChainIkSolverVel_pinv_red();

	virtual int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out);

	virtual int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out){return -1;};

	/** Sets the hand frame */
	void set_eMh(vpHomogeneousMatrix &eMh)
	{this->eMh=eMh; eWh.buildFrom(eMh); hWe.buildFrom(eMh.inverse());}

	/** Whether to use the eef jacobian expressed in the robot base frame (true) or the hand frame jacobian expressed in the hand frame (false, default) */
	void setBaseJacobian(bool flag) {useBaseJacobian=flag;}

private:
	const Chain chain;
	ChainJntToJacSolver jnt2jac;
	Jacobian jac;
	SVD_HH svd;
	std::vector<JntArray> U, V;
	JntArray S, tmp;
	ChainFkSolverPos_recursive *fksolver;
	double eps;
	int maxiter;
	bool useBaseJacobian;

	vpHomogeneousMatrix eMh;        ///< Hand frame
	vpVelocityTwistMatrix eWh, hWe; ///< Twist transformation matrix hand-eef frame
};

}

#endif /* ARM5SOLVERS_H_ */
