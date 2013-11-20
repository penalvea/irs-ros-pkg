/*
 * Odometry.h
 *
 *  Created on: 21/06/2012
 *      Author: mprats
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <mar_ros_bridge/ROSInterface.h>
#include <nav_msgs/Odometry.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>


class Odometry: public ROSMsg<nav_msgs::Odometry> {
public:
	Odometry() {}

	/** Subscriber constructor
	 * Will automatically register a callback and fill the object from the data coming in a ROS topic
	 */
	Odometry(ros::NodeHandle &nh, std::string topic, int queue_length=1) {
		this->setSubscriber(nh, topic, queue_length);
	}

	virtual void publish() {
		msg_.header.stamp=ros::Time::now();
		ROSMsg<nav_msgs::Odometry>::publish();
	}

	vpHomogeneousMatrix getPose();

	vpColVector getTwist();

	virtual ~Odometry() {}
};

#endif /* ODOMETRY_H_ */
