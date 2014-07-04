/*
 * VispUtils: usage example in http://github.com/davidfornas/pcl_manipulation
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
	  addTransform(sMs, parent, child, "0");
}

VispToTF::VispToTF( ){
	  broadcaster_ = new tf::TransformBroadcaster();
}

void VispToTF::addTransform( vpHomogeneousMatrix sMs, std::string parent, std::string child, std::string id){

	  //TODO: It's possible to do further checks.
      if(frames_.count(id)>0){
		  std::cerr << "ID [" << id << "] already in use. It won't be added." << std::endl;
		  return;		  
	  }
	  // Check for transform duplicates, traverse through the map so keep it small.
	  bool duplicate=false;
	  std::string old_id;
	  for( std::map<std::string, Frame>::iterator ii=frames_.begin(); ii!=frames_.end(); ++ii)
	  {
			if( (*ii).second.parent == parent &&  (*ii).second.child == child){
				duplicate=true;
				old_id=(*ii).first;
			}	
	  }
	  //Valid tree structure it's not checked. TF does this check, though.
	  
	  if(duplicate){
		  std::cerr << "Frame from " << child << " to " << parent << " already specified. Reset it with resetTransform(sMs, " << old_id << " )." << std::endl;
		  return;		  
	  }
	  
	  
	  tf::Transform pose=tfTransFromVispHomog(sMs);
	  
	  Frame f; 
	  f.pose=pose;
	  f.parent=parent;
	  f.child=child;
	  
	  frames_[id]=f;
	  
}

void VispToTF::resetTransform( vpHomogeneousMatrix sMs, std::string id){
	  
	  if(frames_.count(id)<1){
		  std::cerr << "Can't reset this item. ID [" << id << "] not found." << std::endl;
		  return;		  
	  }
	  tf::Transform pose=tfTransFromVispHomog(sMs);	  
	  frames_[id].pose=pose;
	  
}

void VispToTF::removeTransform( std::string id){
	  if(frames_.count(id)<1){
		  std::cerr << "Can't delete this item. ID [" << id << "] not found." << std::endl;
		  return;		  
	  }
	  frames_.erase(id);
}

void VispToTF::publish(){

	for( std::map<std::string, Frame>::iterator ii=frames_.begin(); ii!=frames_.end(); ++ii)
		{
			tf::StampedTransform transform((*ii).second.pose, ros::Time::now(), (*ii).second.parent, (*ii).second.child);
			broadcaster_->sendTransform(transform);
		}

}

void VispToTF::print(){

	for( std::map<std::string, Frame>::iterator ii=frames_.begin(); ii!=frames_.end(); ++ii)
		{
			std::cout << "ID string: " << (*ii).first  << std::endl << 
			"Frame -> " << (*ii).second << std::endl;
		}

}

