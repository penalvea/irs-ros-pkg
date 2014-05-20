#ifndef VISPUTILS_H
#define VISPUTILS_H

#include <tf/transform_broadcaster.h>
#include <visp/vpHomogeneousMatrix.h>
#include <string>

struct Frame{
	tf::Transform pose;
	std::string parent, child;
	friend std::ostream& operator<<(std::ostream& out, Frame& x );
};

/** Tool to publish a vpHomogeneousMatrix on the TF tree */
class VispToTF
{

	tf::TransformBroadcaster *broadcaster_;
	std::map<std::string, Frame> frames_;


public:

    /** Create a new publisher with a frame to add to the TF tree */
    VispToTF( vpHomogeneousMatrix sMs, std::string parent, std::string child );
    /** Create a new empty publisher */
    VispToTF();

    /** Add a frame to the publish list */
	void addTransform( vpHomogeneousMatrix sMs, std::string parent, std::string child, std::string id  );
	
	/** Remove a frame from the publish list */
	void removeTransform( std::string id  );
	
	
	void publish();
	void print();

   // ~VispToTF();
};

std::ostream& operator<<(std::ostream& out, Frame& x ) 
{
	  out << "Pose origin  [x: " << x.pose.getOrigin().x() << " y: " << x.pose.getOrigin().y() << " z: " << x.pose.getOrigin().z() << "] "
	  << "Pose rotation [x:" << x.pose.getRotation().x() << " y:" << x.pose.getRotation().y() << " z:" << x.pose.getRotation().z() << " w:" << x.pose.getRotation().w() << "] "
	  << std::endl << "Parent: " << x.parent << " Child: " << x.child << std::endl;
	  return out;
}

#endif
