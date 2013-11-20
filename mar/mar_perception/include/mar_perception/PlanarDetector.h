/*
 * PlanarDetector.h
 *
 *  Created on: 30/08/2012
 *      Author: mprats
 */

#ifndef PLANARDETECTOR_H_
#define PLANARDETECTOR_H_

#include <mar_core/CPerception.h>
#include <visp/vpImage.h>
#include <visp/vpPlanarObjectDetector.h>
#include <vector>

class PlanarDetector: public CPerception {
	 unsigned int height, width;

	 vpImage<unsigned char> *Ireference_, *Icurrent_;
	 std::vector<vpImagePoint> corners_;

public:
    vpPlanarObjectDetector planar;

    PlanarDetector(vpImage<unsigned char> *Icurrent, vpImage<unsigned char> *Ireference);

	void perceive();

	~PlanarDetector();
};


#endif /* PLANARDETECTOR_H_ */
