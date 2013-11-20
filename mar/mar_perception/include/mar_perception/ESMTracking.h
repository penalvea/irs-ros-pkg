#ifndef _ESMTRACKING_H
#define _ESMTRACKING_H

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpHomography.h>
#include <vector>
#include <ros/ros.h>
extern "C" {
#include "ESMlibry.h"
}

#include <boost/shared_ptr.hpp>

/** Class for template tracking based on the ESM library (INRIA)
 */
class ESMTracking {
	// The tracking parameters
	// miter: the number of iterations of the ESM algorithm (>= 1);
	// mprec: the precision of the ESM algorithm (1..10)
	// low precision = 1, high precision = 10;
	// miter and mprec should be chosen depending on the available 
	// computation time; 
	// For low-speed PCs or high video framerate choose low miter and low mprec.
	int miter, mprec;

	// The image read / acquire
	imageStruct Iesm;

	// The global tracking structure
	trackStruct T;

	bool color_input; 	///< True if built from a color image

	vpImage<vpRGBa> *Ic;	///< Image where tracking is performed (color)
	vpImage<unsigned char> *I;	///< Image where tracking is performed (bw)

	void initTracker();
public:
	float pEnd[8];		///< Current position in pixels of the template corners: (0,1), (2,3), (4,5), (6,7)
	vpHomography tHc;		///< Homography of the current pose wrt to the template

	// The template position (upper left corner), size and rotation in the reference image
	// The image coordinates start from (0,0)
	int posx, posy, sizx, sizy;
	float alpha;

public:
	/** Empty constructor */
	ESMTracking() {}

	/** Constructor by knowing the template size and position in the image
	    @param I The image where to perform tracking
	    @param posx, posy, sizx, sizy, alpha: Position, size and rotation of the template to track inside the image
	    @param miter: the number of iterations of the ESM algorithm (>= 1);
	    @param mprec: the precision of the ESM algorithm (1..10). Low precision = 1, high precision = 10
	    miter and mprec should be chosen depending on the available computation time; 
	    For low-speed PCs or high video framerate choose low miter and low mprec.
	 */
	ESMTracking(vpImage<unsigned char> *I, int posx, int posy, int sizx, int sizy, float alpha=0, int miter=8, int mprec=2);
	ESMTracking(vpImage<vpRGBa> *Ic, int posx, int posy, int sizx, int sizy, float alpha=0, int miter=8, int mprec=2);

	/** Constructor without knowing the template size and position. Asks for user initialization
	 *
	 * @param I The image where to perform tracking
	 * @param miter the number of iterations of the ESM algorithm (>= 1);
	 * @param mprec the precision of the ESM algorithm (1..10). Low precision = 1, high precision = 10
	 */
	ESMTracking(vpImage<unsigned char> *I, int miter=8, int mprec=2);
	ESMTracking(vpImage<vpRGBa> *Ic, int miter=8, int mprec=2);

	void manualInit();	///< Displays the image and waits for a manual initialization by the user

	void setInitialEstimation(vpHomography &H);	///< Sets an initial guess of the homography

	virtual void perceive();                      ///< Update the state of the perception

	void updateCornersFromHomography(); ///< Updates the four template corners with the current homography estimation
	void draw(vpImage<vpRGBa> &Ic);  ///< Draw the tracked template on the image
	void draw(vpImage<unsigned char> &I);  ///< Draw the tracked template on the image

	/** Map a point given in pixels wrt the template origin to the current position in image after the homography transform */
	vpImagePoint map(int u, int v);

	/** Returns a vector with the corners of the template in pixels, in the form (i,j,i,j,i,j,i,j)*/
	std::vector<float> getCorners() {
		return std::vector<float>(pEnd, pEnd+8);
	}

	/** Returns the centroid of the template */
	vpImagePoint getCentroid() {
		return map((int)sizx/2.0,(int)sizy/2.0);
	}

	virtual ~ESMTracking();
};
typedef boost::shared_ptr<ESMTracking> ESMTrackingPtr;

#include <mar_msgs/TemplateTrack.h>

class ESMTrackingROSPublisher : public ESMTracking {
	std::string publish_topic_;

	ros::Publisher tt_pub_;
public:
	ESMTrackingROSPublisher(ros::NodeHandle nh, std::string publish_topic, vpImage<unsigned char> *I, int posx, int posy, int sizx, int sizy, float alpha=0, int miter=8, int mprec=2);
	ESMTrackingROSPublisher(ros::NodeHandle nh, std::string publish_topic, vpImage<vpRGBa> *Ic, int posx, int posy, int sizx, int sizy, float alpha=0, int miter=8, int mprec=2);

	virtual void perceive() {
		ESMTracking::perceive();

		//Publish on ROS
		//publish the bounding box, given as the origin corner, angle, width and height
		mar_msgs::TemplateTrack tt;
		tt.corners.push_back(pEnd[1]);
		tt.corners.push_back(pEnd[0]);
		tt.corners.push_back(pEnd[3]);
		tt.corners.push_back(pEnd[2]);
		tt.corners.push_back(pEnd[5]);
		tt.corners.push_back(pEnd[4]);
		tt.corners.push_back(pEnd[7]);
		tt.corners.push_back(pEnd[6]);
		vpImagePoint centroid=map((int)sizx/2.0,(int)sizy/2.0);
		tt.centroid.push_back(centroid.get_i());
		tt.centroid.push_back(centroid.get_j());
		tt.alpha=alpha;
		tt_pub_.publish(tt);
	}

	~ESMTrackingROSPublisher() {}
};
typedef boost::shared_ptr<ESMTrackingROSPublisher> ESMTrackingROSPublisherPtr;


class ESMTrackingROSSubscriber : public ESMTracking {
	std::string tracker_topic_;
	ros::Subscriber bbox_sub_;
public:
	ESMTrackingROSSubscriber(ros::NodeHandle nh, std::string tracker_topic);

	void perceive() {
		//Does nothing. messageReceivedCallback does the work
	}

	void messageReceivedCallback(const mar_msgs::TemplateTrackConstPtr& message) {
		//Fill in fields pEnd, posx, posy, etc.
		memcpy(pEnd, &(message->corners[0]), 8*sizeof(float));
		alpha=message->alpha;
		posx=message->corners[1];
		posy=message->corners[0];
		sizy=vpImagePoint::distance(vpImagePoint(message->corners[0], message->corners[1]), vpImagePoint(message->corners[2], message->corners[3]));
		sizx=vpImagePoint::distance(vpImagePoint(message->corners[0], message->corners[1]), vpImagePoint(message->corners[6], message->corners[7]));
	}

	~ESMTrackingROSSubscriber() {}
};
typedef boost::shared_ptr<ESMTrackingROSSubscriber> ESMTrackingROSSubscriberPtr;


#endif

