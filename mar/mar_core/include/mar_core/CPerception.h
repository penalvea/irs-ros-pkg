#ifndef _CPERCEPTION_H
#define _CPERCEPTION_H

/** Class perception
 * This abstract class represents any kind of information that is obtained from sensors, for example, a contour, grasp points,
 * distance to obstacles, etc.
 */
class CPerception {
	public:	
	virtual void perceive()=0;			///< Update the state of the perception

	virtual ~CPerception() {}
};
#endif
