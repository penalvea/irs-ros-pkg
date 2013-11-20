/*
 * Transform.h
 *
 *  Created on: 01/08/2012
 *      Author: mprats
 */

#ifndef TRANSFORM_H_
#define TRANSFORM_H_

#include <mar_ros_bridge/ROSInterface.h>
#include <geometry_msgs/Transform.h>

#include <visp/vpHomogeneousMatrix.h>

class Transform: public ROSMsg<geometry_msgs::Transform> {
public:
	Transform() {}

	/** Subscriber constructor
	 * Will automatically register a callback and fill the object from the data coming in a ROS topic
	 */
	Transform(ros::NodeHandle &nh, std::string topic, int queue_length=1) {
		this->setSubscriber(nh, topic, queue_length);
	}

	virtual void publish() {
		ROSMsg<geometry_msgs::Transform>::publish();
	}

	vpHomogeneousMatrix getTransform();

	virtual ~Transform() {}
};



#endif /* TRANSFORM_H_ */
