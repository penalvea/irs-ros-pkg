#ifndef ARM5ARM_H
#define ARM5ARM_H

#include <string>
#include <vector>

#include <mar_core/Arm.h>
#include <mar_robot_arm5e/ARM5Solvers.h>

//KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

//ViSP
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>

/** Default ROS topics */
#define DEFAULT_STATE_TOPIC "/arm5e/joint_state_angle"
#define DEFAULT_CONTROL_TOPIC "/arm5e/command_angle"

/** Max allowed discontinuity in the input cartesian velocity.
 * If the euclidean distance between one velocity reference and the following is greated than DISC_EPSILON, the new reference is scaled down.
 */
#define DISC_EPSILON	0.0025

/**
    An API for the ARM5 arm
	@author Mario Prats <mprats@icc.uji.es>
 */
class ARM5Arm : public Arm
{
	ros::NodeHandle nh_;
	ros::Subscriber position_sub;
	ros::Publisher velocity_pub;

	double current;
	KDL::Chain chain;		///< Arm kinematic chain
	KDL::Chain auvarm_chain;	///< Vehicle-arm kinematic chain
	KDL::Chain armhotstab_chain; ///< ARM-hotStab kinematic chain

	KDL::ChainFkSolverPos_recursive *fksolver;
	KDL::ChainIkSolverVel_pinv_red  *ivk_solver;	 //KDL inverse velocity solver of the ARM
	//KDL::ChainIkSolverVel_wdls  *ivk_solver;	 //KDL inverse velocity solver of the ARM
	KDL::ChainIkSolverVel_pinv_red  *auvarm_ivk_solver; //KDL inverse velocity solver of the Vehicle-Arm
	KDL::ChainIkSolverVel_pinv_red  *armhotstab_ivk_solver; //KDL inverse velocity solver of the Arm-hotStab

	bool is_initialized;

	void initKinematicSolvers();

	/** Callback called when JointState messages arrive from the arm */
	void readJointsCallback(const sensor_msgs::JointState::ConstPtr& state);
public:
	/** Void constructor to be used only for computing kinematics, and not for control */
	ARM5Arm();

	/** The constructor as a ROS node that subscribes to ARM5 control topics. Uses default topic names DEFAULT_STATE_TOPIC and DEFAULT_CONTROL_TOPIC */
	ARM5Arm(ros::NodeHandle &nh);

	/** The constructor as a ROS node that subscribes to ARM5 control topics. Takes topic names as arguments */
	ARM5Arm(ros::NodeHandle &nh, std::string state_topic, std::string control_topic);

	/** Initialize the zero offsets of the arm. If it's already initialized, it does nothing
	 * Moves the indicated joints to the limits and sets the arm zero offsets.
	 * Calling this method blocks the calling thread until the arm initialization is finished
	 * @arg q1-q5 if true, initialize the joint. Do not initialize the joint otherwise
	 */
	int init(bool q1=true, bool q2=true, bool q3=true, bool q4=false, bool q5=false);

	/** Force the arm to nitialize the zero offsets, even if it's already initialized
	 * Moves the indicated joints to the limits and sets the arm zero offsets.
	 * Calling this method blocks the calling thread until the arm initialization is finished
	 * @arg q1-q5 if true, initialize the joint. Do not initialize the joint otherwise
	 */
	int forcedInit(bool q1=true, bool q2=true, bool q3=true, bool q4=false, bool q5=false);

	/** Returns true if the arm zero offsets are already initialized */
	bool isInit() {return is_initialized;}

	/** Gets the current end-effector pose w.r.t robot kinematic base (units in meters) */
	virtual int getPosition(vpHomogeneousMatrix &bMe);

	/** Returns the master current sensed at the manipulator */
	double getCurrent() {return current;}

	/** Gets the current joint values */
	virtual int getJointValues(vpColVector &q);

	/** Check joint limits. Returns true if a limit has been reached.
	In that case, the vector inlimit is filled with the indexes of the joints which are in limit
	 */
	bool checkJointLimits(std::vector<int> &inlimit);

	/** Sets the desired joint positions
	 * @param dq desired joint values
	 * @param blocking if true, blocks until the desired position is reached within a tolerance error (tol)
	 * @param tol if the euclidean norm of the difference between the current and desired position is less than tol, consider the task finished
	 * @return
	 */
	virtual int setJointValues(vpColVector &dq, bool blocking=false, double tol=0.05);
	virtual int setJointValues(double dq[5], bool blocking=false, double tol=0.05);
	virtual int setJointValues(vpColVector &dq);


	/** Sets the desired joint velocities */
	virtual int setJointVelocity(vpColVector qdot);
	/** Sets the desired cartesian velocities. Units in meters/s and radians/s */
	virtual int setCartesianVelocity(vpColVector xdot);

	/** Compute the IK of the arm for reaching a given frame wMe (=bMe) */
	vpColVector armIK(vpHomogeneousMatrix &wMe);
	vpColVector armIK(vpHomogeneousMatrix &wMe, vpColVector maxJointLimits, vpColVector minJointLimits);
  vpColVector armIK(vpHomogeneousMatrix &wMe, vpColVector maxJointLimits, vpColVector minJointLimits, vpColVector initial_joints);


	/** Compute the IK of the vehicle-arm for reaching a given frame wMe */
	vpColVector vehicleArmIK(vpHomogeneousMatrix &wMe);
        vpColVector vehicleArmIK(vpHomogeneousMatrix &wMe, vpColVector masJointLimits, vpColVector minJointLimits);

	/** Compute the IK of the arm-hotStab for reaching a given frame wMe (=bMe) */
	vpColVector armHotStabIK(vpHomogeneousMatrix &wMe);

	/** Compute the most suitable camera-object relative position for reaching a given frame wMg
	 * @param cMb is the camera-to-armbase calibration
	 * @param oMg is the grasp frame wrt the object frame
	 * @param wMg is the grasp frame in world coordinates
	 * @returns cMo the best camera-object transformation
	 */
	vpHomogeneousMatrix compute_best_cMo(vpHomogeneousMatrix cMb, vpHomogeneousMatrix oMg, vpHomogeneousMatrix eMh, vpHomogeneousMatrix wMg);

	~ARM5Arm();

	/** Sets the reference frame of the jacobian for the IK solvers
	 * @param flag if true, use the base jacobian. If false, use end-effector jacobian
	 */
	void setBaseJacobian(bool flag) {if (ivk_solver!=NULL) ivk_solver->setBaseJacobian(flag);}

	/** Sets the end-effector to hand transformation
	 * @oaram eMh the hand frame with respect to the end-effector frame
	 */
	void set_eMh(vpHomogeneousMatrix &eMh) {
		if (ivk_solver!=NULL) ivk_solver->set_eMh(eMh);
		if (auvarm_ivk_solver!=NULL) auvarm_ivk_solver->set_eMh(eMh);
	}

	/** Computes the forward kinematics of the arm */
	vpHomogeneousMatrix directKinematics(vpColVector q);

public:
	vpColVector qmax, qmin, qrange, qamax, qamin;   ///< Allowed joint range
	vpColVector q; ///< Current joint values
};
typedef boost::shared_ptr<ARM5Arm> ARM5ArmPtr;


//TODO: calibrate
static const double ARM5EZeroAbsTicks[5]={10841,0,0,0,0};

class InitCSIP
{
public:
	InitCSIP(ros::NodeHandle &nh, bool q1=true, bool q2=true, bool q3=true, bool q4=false, bool q5=false);

	bool initAuto;		//Whether to initialize automatically without user teleoperation
	bool initAxis[5];		//axis to initialize
	bool initAxisDone[5];		//axis already initialized
	int AxisDir[5];		//Direction to move (+1 or -1)
	double initAxisThreshold[5];    //Current threshold for each joint initialization
	int position[5];		//Current joint position
	int limitPosition[5];		//Joint position at the limit
	double current;		//Current the arm is consuming

	int activeAxis;

	ros::Time limitDetected;

	ros::Publisher vel_pub_;	//Publish arm command velocity
	ros::Subscriber js_sub_;	//Subscribe to arm joint state

	int callSetZeroService();
	void autoInitVel();

private:
	void stateCallback(const sensor_msgs::JointState::ConstPtr& state);

	double scale_;

	ros::ServiceClient setZeroClient;

};

#endif
