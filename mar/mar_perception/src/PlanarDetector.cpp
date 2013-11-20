/*
 * PlanarDetector.cpp
 *
 *  Created on: 30/08/2012
 *      Author: mprats
 */

#include <mar_perception/PlanarDetector.h>

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>

PlanarDetector::PlanarDetector(vpImage<unsigned char> *Icurrent, vpImage<unsigned char> *Ireference) {
	Ireference_=Ireference;
	Icurrent_=Icurrent;

	//Select a part of the image by clincking on two points which define a rectangle
	corners_.push_back(vpImagePoint(0,0));
	corners_.push_back(vpImagePoint(Ireference->getRows()-1,Ireference->getCols()-1));

	//Build the reference points (and train the classifier).
	height = (unsigned int)(corners_[1].get_i() - corners_[0].get_i());
	width = (unsigned int)(corners_[1].get_j() - corners_[0].get_j());
	std::cerr << "Image size: " << Ireference->getRows() << " " << Ireference->getCols() << std::endl;
	std::cerr << "height and width of template: " << height << " " << width << std::endl;
	int num = planar.buildReference(*Ireference, corners_[0], height, width);
	std::cerr << num << std::endl;
}

void PlanarDetector::perceive() {
	//Match points between the reference points and the current points computed in the current image.
	bool isDetected;
	isDetected = planar.matchPoint(*Icurrent_);

	//Display the matched points
	if(isDetected){
		planar.display(*Ireference_, *Icurrent_);
		vpHomography homography;
		planar.getHomography(homography);
		std::cerr << "Homography: " << std::endl << homography << std::endl;
	}
	else{
		std::cerr << "planar surface not detected in the current image" << std::endl;
	}
}

PlanarDetector::~PlanarDetector() {

}


