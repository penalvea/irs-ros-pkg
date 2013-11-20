/*
 * Wrench.h
 *
 *  Created on: 01/08/2012
 *      Author: mprats
 */

#ifndef WRENCH_H_
#define WRENCH_H_

#include <mar_ros_bridge/ROSInterface.h>
#include <geometry_msgs/Wrench.h>

#include <visp/vpColVector.h>


class Wrench: public ROSMsg<geometry_msgs::Wrench> {
public:
	Wrench() {}

	/** Subscriber constructor
	 * Will automatically register a callback and fill the object from the data coming in a ROS topic
	 */
	Wrench(ros::NodeHandle &nh, std::string topic, int queue_length=1) {
		this->setSubscriber(nh, topic, queue_length);
	}

	virtual void publish() {
		ROSMsg<geometry_msgs::Wrench>::publish();
	}

	virtual vpColVector getWrench();

	virtual ~Wrench() {}
};

/** Interface for a ROS force sensor that publishes a geometry_msgs::Wrench topic
 */
class ForceSensor: public Wrench {
	vpColVector offsets;
public:
	ForceSensor() {}

	ForceSensor(ros::NodeHandle &nh, std::string topic, int queue_length=1): Wrench(nh,topic, queue_length) {
		offsets.resize(6);
		offsets=0;
	}

	virtual void publish() {
		ROSMsg<geometry_msgs::Wrench>::publish();
	}

	/** Sets the zero offsets.
	 * When called, the current sensor reading is used as an offset to future readings
	 */
	void setZero() {
		if (isInitialized()) offsets=Wrench::getWrench();
		else {
			ROS_WARN("ForceSensor::setZero(): Subscriber still didn't receive data");
		}
	}

	virtual vpColVector getWrench() {
		return Wrench::getWrench()-offsets;
	}

	virtual ~ForceSensor() {}

};

#endif /* WRENCH_H_ */
