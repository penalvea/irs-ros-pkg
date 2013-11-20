/*
 * MotionEstimator.h
 *
 *  Created on: 24/05/2012
 *      Author: mprats
 */

#ifndef MOTIONESTIMATOR_H_
#define MOTIONESTIMATOR_H_

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImagePoint.h>
#include <visp/vpCameraParameters.h>

#include <boost/shared_ptr.hpp>

#include <mar_core/CPerception.h>
#include <mar_perception/ESMTracking.h>

/** Abstract class for motion estimators
 *  At a given time, estimate the pose with respect to a reference frame
 */
class MotionEstimator : public CPerception {
protected:
	vpHomogeneousMatrix rMc;	///current pose with respect to the reference one
public:
	MotionEstimator() {}

	virtual void perceive()=0;

	vpHomogeneousMatrix getMotionEstimation() {return rMc;}

	/** Reset the motion estimator. Takes the current pose as the reference one */
	virtual void reset() {rMc.setIdentity();}

	virtual ~MotionEstimator() {}
};
typedef boost::shared_ptr<MotionEstimator> MotionEstimatorPtr;

/** A planar motion estimator based on ESM Tracking
 *
 */
class ESMMotionEstimator : public MotionEstimator {
	boost::shared_ptr<ESMTracking> esm_;
	vpImagePoint initial_centroid_, current_centroid_;
	float initial_alpha_, current_alpha_;

	vpCameraParameters K_;	///< Camera parameter
	double z_;	///<Distance in meters to the template being tracked by ESM

public:
	ESMMotionEstimator(boost::shared_ptr<ESMTracking> esm, vpCameraParameters K): MotionEstimator(), esm_(esm), K_(K) {
		//by default set z=1
		setZ(1.0);
		reset();
	}

	virtual void perceive() {
		current_centroid_=esm_->getCentroid();
		current_alpha_=esm_->alpha;
		rMc.buildFrom((initial_centroid_.get_j()-current_centroid_.get_j())*z_/K_.get_px(),
					  (initial_centroid_.get_i()-current_centroid_.get_i())*z_/K_.get_py(),
					  0, 0, 0, initial_alpha_-current_alpha_);
	}

	virtual void reset() {
		MotionEstimator::reset();
		initial_centroid_=esm_->getCentroid();
		initial_alpha_=esm_->alpha;
	}

	/** Sets the distance in meters to the template being tracked.
	 * For estimating motion with this method, the distance to the target needs to be known
	 * @param z distance to the target
	 */
	void setZ(double z) {
		z_=z;
	}

	virtual ~ESMMotionEstimator() {}
};
typedef boost::shared_ptr<ESMMotionEstimator> ESMMotionEstimatorPtr;

#endif /* MOTIONESTIMATOR_H_ */
