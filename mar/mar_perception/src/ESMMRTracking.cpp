#include <mar_perception/ESMMRTracking.h>
#include <visp/vpDisplay.h>
#include <stdlib.h>

ESMMRTracking::ESMMRTracking(vpImage<unsigned char> *I, int posx, int posy, int sizx, int sizy, int rx, int ry, int miter, int mprec) {
	this->I=I;
	this->posx=posx;
	this->posy=posy;
	this->sizx=sizx;
	this->sizy=sizy;
	this->miter=miter;
	this->mprec=mprec;
	this->rx=rx;
	this->ry=ry;

	// Load Iesm with initial/reference image
	std::cerr << "Image copy" << std::endl;
	Iesm.cols=I->getWidth();
	Iesm.rows=I->getHeight();
#ifndef __LP64__
	Iesm.clrs=1;
#endif
	Iesm.data=(float*)malloc(I->getNumberOfPixel()*sizeof(float));
	for (unsigned int i=0;i<I->getNumberOfPixel();i++)
		Iesm.data[i]=(float)I->bitmap[i];

	cellsizex=(int)(sizx/rx);
	cellsizey=(int)(sizy/ry);
	for (int x=0; x<rx; x++) {
	  for (int y=0; y<ry; y++) {
		patches.push_back(new ESMMRPatch(&Iesm, posx+x*cellsizex, posy+y*cellsizey, cellsizex, cellsizey, miter, mprec));
	  }
	}
}

void ESMMRTracking::perceive() {
    //copy I to Iesm 
    for (unsigned int i=0;i<I->getNumberOfPixel();i++)
        Iesm.data[i]=(float)I->bitmap[i];

    for (unsigned int p=0; p<patches.size(); p++) {
	// Perform the tracking for the ith patch
	if (MakeTrack (&(patches[p]->T), &Iesm)) {
		std::cerr << "ESMMRTracking::perceive() ERROR: Tracking failed in patch number " << p << std::endl;
		exit(0);
	}

	float pIni[8];
	float *H=patches[p]->T.homog;
        for (int i=0; i<3; i++)
	   for (int j=0; j<3; j++)
		patches[p]->tHc[i][j]=H[i*3+j];
	pIni[0] = 0;    pIni[1] = 0;      pIni[2] = cellsizex - 1; pIni[3] = 0;
	pIni[4] = cellsizex-1; pIni[5] = cellsizey - 1; pIni[6] = 0;      pIni[7] = cellsizey - 1;

	for (int i = 0; i < 4; i++) {
		patches[p]->pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
			    (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
		patches[p]->pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
			    (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
	}
    }
    
}

void ESMMRTracking::draw(vpImage<vpRGBa> &Ic) {
    for (unsigned int i=0; i<patches.size(); i++) {
	    vpDisplay::displayLine (Ic, (int)patches[i]->pEnd[1], (int)patches[i]->pEnd[0], (int)patches[i]->pEnd[3], (int)patches[i]->pEnd[2],vpColor::green,3);
	    vpDisplay::displayLine (Ic, (int)patches[i]->pEnd[3], (int)patches[i]->pEnd[2], (int)patches[i]->pEnd[5], (int)patches[i]->pEnd[4],vpColor::green,3);
	    vpDisplay::displayLine (Ic, (int)patches[i]->pEnd[5], (int)patches[i]->pEnd[4], (int)patches[i]->pEnd[7], (int)patches[i]->pEnd[6],vpColor::green,3);
	    vpDisplay::displayLine (Ic, (int)patches[i]->pEnd[7], (int)patches[i]->pEnd[6], (int)patches[i]->pEnd[1], (int)patches[i]->pEnd[0],vpColor::green,3);
    }

}

void ESMMRTracking::draw(vpImage<unsigned char> &I) {
    for (unsigned int i=0; i<patches.size(); i++) {
	    vpDisplay::displayLine (I, (int)patches[i]->pEnd[1], (int)patches[i]->pEnd[0], (int)patches[i]->pEnd[3], (int)patches[i]->pEnd[2],vpColor::green,3);
	    vpDisplay::displayLine (I, (int)patches[i]->pEnd[3], (int)patches[i]->pEnd[2], (int)patches[i]->pEnd[5], (int)patches[i]->pEnd[4],vpColor::green,3);
	    vpDisplay::displayLine (I, (int)patches[i]->pEnd[5], (int)patches[i]->pEnd[4], (int)patches[i]->pEnd[7], (int)patches[i]->pEnd[6],vpColor::green,3);
	    vpDisplay::displayLine (I, (int)patches[i]->pEnd[7], (int)patches[i]->pEnd[6], (int)patches[i]->pEnd[1], (int)patches[i]->pEnd[0],vpColor::green,3);
    }
}

/** Map a point given in pixels wrt the template origin to the current position in image after the homography transform */
vpImagePoint ESMMRTracking::map(int u, int v) {
	float *H=patches[0]->T.homog;
	return vpImagePoint( (H[3]*u + H[4]*v + H[5]) / (H[6]*u + H[7]*v + H[8]),
			     (H[0]*u + H[1]*v + H[2]) / (H[6]*u + H[7]*v + H[8]));	
}

ESMMRTracking::~ESMMRTracking() {
}
