#ifndef CACTION_H
#define CACTION_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

class CAction {
protected:
	boost::shared_ptr<vpImage<vpRGBa> > Idraw_;
	bool drawing_enabled;

public:
	CAction() {drawing_enabled=false;}

	virtual int doAction()=0;	///< Perform the action

	/** Enables drawing. If enabled, the draw() method can be called on the action main loop */
	void enableDrawing(boost::shared_ptr<vpImage<vpRGBa> > I) {Idraw_=I; drawing_enabled=true;}

	/** Disables drawing */
	void disableDrawing() {Idraw_.reset(); drawing_enabled=false;}

	/** Draw the progress of the action on an image. To be overloaded by specialized classes */
	virtual void draw() {}

	virtual ~CAction() {};
};

#endif
