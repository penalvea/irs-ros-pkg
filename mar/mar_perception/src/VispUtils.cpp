/*
 * Transform.cpp
 *
 *  Created on: 24/04/2014
 *      Author: dfornas
 */
#include <mar_perception/VispUtils.h>
//#include <kdl/frames.hpp>
#include <tf/transform_broadcaster.h>
#include <visp/vpHomogeneousMatrix.h>
#include <stdlib.h>


VispToTF::VispToTF( vpHomogeneousMatrix sMs, std::string parent, std::string child){
	  broadcaster_ = new tf::TransformBroadcaster();
	  setTransform(sMs, parent, child);
}

void VispToTF::setTransform( vpHomogeneousMatrix sMs, std::string parent, std::string child){

	  parent_=parent;
	  child_=child;

	  vpTranslationVector trans;
	  sMs.extract(trans);
	  tf::Vector3 translation(trans[0],trans[1],trans[2]);

	  vpQuaternionVector quat;
	  sMs.extract(quat);
	  tf::Quaternion rotation( quat.x(), quat.y(), quat.z(), quat.w());
	  pose_.setOrigin(translation);   pose_.setRotation(rotation);

}

void VispToTF::publish(){

    tf::StampedTransform transform(pose_, ros::Time::now(), parent_, child_);
    broadcaster_->sendTransform(transform);
}
