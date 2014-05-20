#ifndef VISPUTILS_H
#define VISPUTILS_H

#include <tf/transform_broadcaster.h>
#include <visp/vpHomogeneousMatrix.h>
#include <string>


/** Tool to publish a vpHomogeneousMatrix on the TF tree */
class VispToTF
{

	tf::TransformBroadcaster *broadcaster_;
	tf::Transform pose_;
	std::string parent_, child_;

public:


    /** Constructor from a VISP homoggeneous matrix and the parent and child ids of the frame */
	VispToTF( vpHomogeneousMatrix sMs, std::string parent, std::string child );

	void setTransform( vpHomogeneousMatrix sMs, std::string parent, std::string child  );
	void publish();

   // ~VispToTF();
};

#endif
