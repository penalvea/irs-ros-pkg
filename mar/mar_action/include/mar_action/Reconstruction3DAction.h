#ifndef RECONSTRUCTION3DACTION_H
#define RECONSTRUCTION3DACTION_H

#include <mar_core/CAction.h>

#include <mar_perception/VirtualImage.h>
#include <mar_core/Arm.h>
#include <mar_perception/ESMTracking.h>
#include <mar_perception/Reconstruction3D.h>
#include <mar_perception/MotionEstimator.h>

#include <visp/vpDisplay.h>
#include <visp/vpColVector.h>

#include <boost/shared_ptr.hpp>

/** Maximum joint velocity to send to the arm */
#define DEFAULT_MAX_JOINT_VELOCITY 0.05

/** Stops the scanning when the position error of the joint with the maximum error is below POSITION_TOLERANCE radians */
#define DEFAULT_POSITION_TOLERANCE 0.02

class Reconstruction3DAction : public CAction {
	typedef enum {SUCCESS, ERROR} output_t;

	boost::shared_ptr<vpImage<vpRGBa> > I_;

	//Pointers to perception classes
	VirtualImagePtr grabber_;
	ArmPtr robot_;
	ESMTrackingPtr tracker_;
	ArmLaserReconstruction3DPtr rec_;
	MotionEstimatorPtr mest_;

	vpColVector vp_scan_initial_posture_;
	vpColVector vp_scan_final_posture_;

	double max_joint_velocity_;
	double position_tolerance_;

	bool fixed_base_; ///< If fixed base, do not perform tracking
	bool offline_;	///< If offline, do not try to move the robot arm

public:	
	Reconstruction3DAction(boost::shared_ptr<vpImage<vpRGBa> > I, VirtualImagePtr grabber, ArmPtr robot, ESMTrackingPtr tracker, ArmLaserReconstruction3DPtr rec, MotionEstimatorPtr mest )
	: CAction(), I_(I), grabber_(grabber), robot_(robot), tracker_(tracker), rec_(rec), mest_(mest), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(DEFAULT_POSITION_TOLERANCE),
	  fixed_base_(false), offline_(false) {
	}

	Reconstruction3DAction(boost::shared_ptr<vpImage<vpRGBa> > I, VirtualImagePtr grabber, ArmLaserReconstruction3DPtr rec)
	: CAction(), I_(I), grabber_(grabber), rec_(rec), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(DEFAULT_POSITION_TOLERANCE),
	  fixed_base_(true), offline_(true) {
	}

	Reconstruction3DAction(boost::shared_ptr<vpImage<vpRGBa> > I, VirtualImagePtr grabber, ESMTrackingPtr tracker, ArmLaserReconstruction3DPtr rec, MotionEstimatorPtr mest )
	: CAction(), I_(I), grabber_(grabber), tracker_(tracker), rec_(rec), mest_(mest), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(DEFAULT_POSITION_TOLERANCE),
	  fixed_base_(false), offline_(true) {
	}

	Reconstruction3DAction(boost::shared_ptr<vpImage<vpRGBa> > I, VirtualImagePtr grabber, ArmPtr robot, ArmLaserReconstruction3DPtr rec )
	: CAction(), I_(I), grabber_(grabber), robot_(robot), rec_(rec), max_joint_velocity_(DEFAULT_MAX_JOINT_VELOCITY), position_tolerance_(DEFAULT_POSITION_TOLERANCE),
	  fixed_base_(true), offline_(false) {
	}

	int doAction();	///< Perform the action
	
	virtual void draw();

	/** Set the joint values where to start the scan */
	void setInitialPosture(vpColVector v) {vp_scan_initial_posture_=v;}

	/** Set the joint values where to finish the scan */
	void setFinalPosture(vpColVector v) {vp_scan_final_posture_=v;}

	void setMaxJointVelocity(double v) {max_joint_velocity_=v;}
	void setTolerance(double t) {position_tolerance_=t;}

	/** If set to true, the action does not perform tracking */
	void setFixedBase(bool flag) {fixed_base_=flag;}

	/** If set to true, the action does not try to access the robot arm */
	void setOffline(bool flag) {offline_=flag;}

	virtual ~Reconstruction3DAction() {};
};

typedef boost::shared_ptr<Reconstruction3DAction> Reconstruction3DActionPtr;

#endif
