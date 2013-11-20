#ifndef _ESMMRMRTRACKING_H
#define _ESMMRMRTRACKING_H

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpHomography.h>
extern "C" {
  #include "ESMlibry.h"
}
#include <stdlib.h>
#include <vector>


class ESMMRPatch {
	public:
		float pEnd[8];		///< Current position in pixels of the template corners: (0,1), (2,3), (4,5), (6,7)
		vpHomography tHc;		///< Homography of the current pose wrt to the template

		bool enabled; //true if the patch is enabled for tracking
		trackStruct T; // The tracking structure

		ESMMRPatch(imageStruct *Iesm, int posx, int posy, int sizx, int sizy, int miter, int mprec)
		{ 
		  enabled=true;
		  std::cerr << "Creating patch at pos " << posx << " " << posy << " with size " << sizx << " " << sizy << std::endl;
		  if (MallTrack (&T, Iesm, posx, posy, sizx, sizy, miter, mprec)) {
			std::cerr << "ESMMRTracking::ESMMRTracking ERROR: Cannot allocate ESMMR Tracking structure" << std::endl;
			exit(0);
		  }
		  std::cerr << "Patch created" << std::endl;
		}

		~ESMMRPatch()
		  {std::cerr << "Patch destroyed" << std::endl;
		   FreeTrack (&T);}
};

/** Class for template tracking based on the ESM library (INRIA). This class add multirresolution (MR) functionality to the basic library.
 */
class ESMMRTracking {
	// The tracking parameters
	// miter: the number of iterations of the ESMMR algorithm (>= 1);
	// mprec: the precision of the ESMMR algorithm (1..10)
	// low precision = 1, high precision = 10;
	// miter and mprec should be chosen depending on the available 
	// computation time; 
	// For low-speed PCs or high video framerate choose low miter and low mprec.
	// rx and ry are the tracking cells in x and y direction
	int miter, mprec, rx, ry, cellsizex, cellsizey;

	// The window position (upper left corner) and size
  	// The image coordinates start from (0,0)
	int posx, posy, sizx, sizy;

	// The image read / acquire
	imageStruct Iesm;

	std::vector<ESMMRPatch*> patches;

	vpImage<unsigned char> *I;	///< Image where tracking is performed

        public:
	/** The tracking parameters
	    @param I The image where to perform tracking
	    @param posx, posy, sizx, sizy: Position and size of the template to track inside the image
	    @param miter: the number of iterations of the ESMMR algorithm (>= 1);
	    @param mprec: the precision of the ESMMR algorithm (1..10). Low precision = 1, high precision = 10
	    miter and mprec should be chosen depending on the available computation time; 
	    For low-speed PCs or high video framerate choose low miter and low mprec.
	*/
	ESMMRTracking(vpImage<unsigned char> *I, int posx, int posy, int sizx, int sizy, int rx=1, int ry=1, int miter=8, int mprec=2);

        virtual void perceive();                      ///< Update the state of the perception

	void draw(vpImage<vpRGBa> &Ic);  ///< Draw the tracked template on the image
	void draw(vpImage<unsigned char> &I);  ///< Draw the tracked template on the image

	/** Map a point given in pixels wrt the template origin to the current position in image after the homography transform */
	vpImagePoint map(int u, int v);

	~ESMMRTracking();
};
#endif

