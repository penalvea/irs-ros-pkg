/*
 * ROSInterface.h
 *
 *  Created on: 21/06/2012
 *      Author: mprats
 */

#ifndef ROSINTERFACE_H_
#define ROSINTERFACE_H_

#include <ros/ros.h>
#include <string>

class ROSInterface {
protected:
	ros::NodeHandle nh_;
	std::string topic_;

public:
	ROSInterface(ros::NodeHandle &nh, std::string topic): nh_(nh), topic_(topic) {}

	virtual ~ROSInterface(){}
};


template<class T>
class ROSSubscriber: public ROSInterface {
protected:
	ros::Subscriber sub_;

	T *msg_;
	bool initialized_;
public:
	ROSSubscriber(): initialized_(false) {}

	ROSSubscriber(T *msg, ros::NodeHandle &nh, std::string topic, int queue_length=1): ROSInterface(nh, topic), initialized_(false) {
		msg_=msg;
		ROS_INFO("Subscriber on topic %s",topic.c_str());
		sub_ = nh.subscribe<T>(topic, queue_length, &ROSSubscriber::msgCallback, this);
		if (sub_==ros::Subscriber()) {
			ROS_ERROR("createSubscriber cannot subscribe to topic %s",topic.c_str());
		}
	}

	void msgCallback(const boost::shared_ptr<T const>& odom) {
			*msg_=*odom;
			initialized_=true;
	}

	bool isInitialized() {return initialized_;}

	virtual ~ROSSubscriber() {}
};

template<class T>
class ROSPublisher: public ROSInterface {
protected:
	ros::Publisher pub_;

	T *msg_;
public:
	ROSPublisher(T *msg, ros::NodeHandle &nh, std::string topic, int queue_length=1): ROSInterface(nh, topic) {
		msg_=msg;
		pub_ = nh.advertise<T>(topic, queue_length);
	}

	virtual void publish() {
		if (pub_ && topic_!="")
			pub_.publish(*msg_);
	}

	virtual ~ROSPublisher() {}
};

template<class T>
class ROSMsg {
protected:
	boost::shared_ptr<ROSSubscriber<T> > subscriber_;
	boost::shared_ptr<ROSPublisher<T> > publisher_;

	T msg_;
public:
	bool setSubscriber(ros::NodeHandle &nh, std::string topic, int queue_length=1) {
		subscriber_=boost::shared_ptr<ROSSubscriber<T> >(new ROSSubscriber<T>(&msg_, nh, topic, queue_length));
		return (subscriber_);
	}

	bool setPublisher(ros::NodeHandle &nh, std::string topic, int queue_length=1) {
		publisher_=boost::shared_ptr<ROSPublisher<T> >(new ROSPublisher<T>(&msg_, nh, topic, queue_length));
		return (publisher_);
	}

	virtual void publish() {
		publisher_->publish();
	}

	T *getMsg() {return &msg_;}

	/** @returns true if the object is a subscriber and has already received data on the input topicr
	 */
	bool isInitialized() {if (subscriber_) return subscriber_->isInitialized(); else return false;}

	/** Waits until data is received in the input topic of a subscriber
	 *  This method blocks the calling thread until data is received by the subscriber
	 *  @returns true if data received, false otherwise (e.g ros shutdown)
	 * */
	bool waitForInit() {
		if (!subscriber_) return true;

		ros::Rate r(5);
		while (!isInitialized() && ros::ok()) {
			ros::spinOnce();
			r.sleep();
		}
		return ros::ok();
	}

	virtual ~ROSMsg() {}
};

#endif /* ROSINTERFACE_H_ */
