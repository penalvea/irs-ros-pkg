#ifndef ARM_H
#define ARM_H

#include <mar_core/CRobot.h>

#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <boost/shared_ptr.hpp>

/**
An abstract interface for manipulators
	@author Mario Prats <mprats@icc.uji.es>
*/
class Arm : public CRobot
{
	
public:
    /** The constructor */
    Arm() {}

    virtual int connect() {return 0;}
    virtual int init() {return 0;}
    virtual int disconnect() {return 0;}

    /** Gets the current end-effector pose w.r.t robot kinematic base (units in meters) */
    virtual int getPosition(vpHomogeneousMatrix &bMe)=0;

    /** Gets the current joint values */
    virtual int getJointValues(vpColVector &q)=0;

    /** Sets the desired joint positions
     * @param dq desired joint values
     * @param blocking if true, blocks until the desired position is reached within a tolerance error (tol)
     * @param tol if the euclidean norm of the difference between the current and desired position is less than tol, consider the task finished
     * @return
     */
    virtual int setJointValues(vpColVector &dq, bool blocking=false, double tol=0.05)=0;

    /** Sets the desired joint velocities */
    virtual int setJointVelocity(vpColVector qdot)=0;
    /** Sets the desired cartesian velocities. Units in meters/s and radians/s */
    virtual int setCartesianVelocity(vpColVector qdot)=0;

    virtual ~Arm() {}
};
typedef boost::shared_ptr<Arm> ArmPtr;

#endif
