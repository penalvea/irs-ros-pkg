#ifndef CROBOT_H
#define CROBOT_H

#include <boost/shared_ptr.hpp>

class CRobot {
	public:
	/** Specific code for connecting to the robot */
	virtual int connect()=0;

	/** Specific code for robot initializing */
	virtual int init()=0;

	/** Specific code for disconnecting from the robot */
	virtual int disconnect()=0;

	virtual ~CRobot() {}
};
typedef boost::shared_ptr<CRobot> CRobotPtr;

#endif
