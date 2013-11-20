#include <mar_perception/ESMTracking.h>
#include <visp/vpDisplay.h>
#include <stdlib.h>

#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImageConvert.h>
#include <math.h>

ESMTracking::ESMTracking(vpImage<unsigned char> *I, int posx, int posy, int sizx, int sizy, float alpha, int miter, int mprec) {
	this->I=I;
	this->Ic=NULL;
	color_input=false;
	this->posx=posx;
	this->posy=posy;
	this->sizx=sizx;
	this->sizy=sizy;
	this->alpha=alpha;
	this->miter=miter;
	this->mprec=mprec;
	initTracker();
}

ESMTracking::ESMTracking(vpImage<vpRGBa> *Ic, int posx, int posy, int sizx, int sizy, float alpha, int miter, int mprec) {
	this->Ic=Ic;
	I=new vpImage<unsigned char>(Ic->getRows(), Ic->getCols());
	vpImageConvert::convert(*Ic,*I);
	color_input=true;
	this->posx=posx;
	this->posy=posy;
	this->sizx=sizx;
	this->sizy=sizy;
	this->alpha=alpha;
	this->miter=miter;
	this->mprec=mprec;
	initTracker();
}

ESMTracking::ESMTracking(vpImage<unsigned char> *I, int miter, int mprec) {
	this->I=I;
	this->Ic=NULL;
	color_input=false;
	this->miter=miter;
	this->mprec=mprec;
	manualInit();
	initTracker();
}

ESMTracking::ESMTracking(vpImage<vpRGBa> *Ic, int miter, int mprec) {
	this->Ic=Ic;
	I=new vpImage<unsigned char>(Ic->getRows(), Ic->getCols());
	vpImageConvert::convert(*Ic,*I);
	color_input=true;
	this->miter=miter;
	this->mprec=mprec;
	manualInit();
	initTracker();
}

void ESMTracking::initTracker() {
	// Load Iesm with initial/reference image. Need to rotate current image with -alpha before loading the template
	// This is because ESM library can only initialize a zero-orientation template 
	//std::cerr << "Initializing the tracking template... " << std::endl;
	vpImage<unsigned char> Iwarp(I->getHeight(), I->getWidth());
	vpHomogeneousMatrix iMt(posx, posy,0,0,0,0);
	vpHomogeneousMatrix tMi=iMt.inverse();
	vpHomogeneousMatrix Ralpha(0,0,0,0,0,alpha);

	vpColVector p(4), pwarp(4);
	for (int r=posy; r<posy+sizy; r++) {
		for (int c=posx; c<posx+sizx; c++) {
			//transform (r,c) according to -alpha
			//std::cerr << "Warping point " << c << " " << r << std::endl;
			pwarp[0]=c; pwarp[1]=r; pwarp[2]=0; pwarp[3]=1;
			p= iMt * Ralpha * tMi * pwarp;
			//pwarp= iMt * Ralpha * tMi * p;
			if ((int)round(p[0])>=0 && (int)round(p[0])<I->getWidth() && (int)round(p[1])>=0 && (int)round(p[1])<I->getHeight()) {
				//warped point is inside the image: draw
				//std::cerr << "Point " << p[0] << " " << p[1] << " warps to " << pwarp[0] << " " << pwarp[1] << " is inside the limits" << std::endl;
				Iwarp[r][c]=(*I)[(int)round(p[1])][(int)round(p[0])];
			}
		}
	}
	//std::cerr << "Template initialized... " << std::endl;

	Iesm.cols=Iwarp.getWidth();
	Iesm.rows=Iwarp.getHeight();
#ifndef __LP64__
	Iesm.clrs=1;
#else
#pragma message ESMTracking build for 64 bits
#endif

	std::cerr << "malloc of size " << Iesm.cols << "x" << Iesm.rows <<  std::endl;
	Iesm.data=(float*)malloc(Iwarp.getNumberOfPixel()*sizeof(float));
	//std::cerr << "copy" << std::endl;
	for (unsigned int i=0;i<Iwarp.getNumberOfPixel();i++)
		Iesm.data[i]=(float)Iwarp.bitmap[i];

	std::cerr << "malltrack: " << posx << " " << posy << " "<<  sizx <<"x"<< sizy <<std::endl;
	// Memory allocation for the tracking
	if (MallTrack (&T, &Iesm, posx, posy, sizx, sizy, miter, mprec)) {
		std::cerr << "ESMTracking::ESMTracking ERROR: Cannot allocate ESM Tracking structure" << std::endl;
		exit(0);
	}
	//std::cerr << "done" << std::endl;

	//Setting initial homography rotation
	T.homog[0]=Ralpha[0][0];
	T.homog[1]=Ralpha[0][1];
	T.homog[3]=Ralpha[1][0];
	T.homog[4]=Ralpha[1][1];
	std::cerr << "Initial homog: ";
	for (int i=0; i<9; i++) {
		std::cerr << T.homog[i] << " ";
	}
	std::cerr << std::endl;
}


void ESMTracking::manualInit() {
	vpImagePoint clicks[3],cursor;
	int nclicks=0;
	if (color_input) {
		while (nclicks<3) {
			vpImageConvert::convert(*Ic,*I);
			vpDisplay::display(*Ic);
			if (nclicks>=1) {
				//draw click point
				vpDisplay::displayCross(*Ic,clicks[0],10,vpColor::red,3);
			}
			if (nclicks>=2) {
				//draw 2nd point, Y line and bounding box
				vpDisplay::displayCross(*Ic,clicks[1],10,vpColor::red,3);
				vpDisplay::displayLine(*Ic,clicks[0],clicks[1],vpColor::green,3);
				//get mouse pointer and draw bounding box
				//vpDisplay::getPointerPosition(Ic,cursor);

			}
			if (vpDisplay::getClick(*Ic,clicks[nclicks],false)) {
				//click done
				nclicks++;
			}
			vpDisplay::flush(*Ic);
		}
		vpDisplay::displayCross(*Ic,clicks[2],10,vpColor::red,3);
		vpDisplay::flush(*Ic);
	} else {
		while (nclicks<3) {
			vpDisplay::display(*Ic);
			if (nclicks>=1) {
				//draw click point
				vpDisplay::displayCross(*Ic,clicks[0],10,vpColor::red,3);
			}
			if (nclicks>=2) {
				//draw 2nd point, Y line and bounding box
				vpDisplay::displayCross(*Ic,clicks[1],10,vpColor::red,3);
				vpDisplay::displayLine(*Ic,clicks[0],clicks[1],vpColor::green,3);
				//get mouse pointer and draw bounding box
				//vpDisplay::getPointerPosition(Ic,cursor);

			}
			if (vpDisplay::getClick(*Ic,clicks[nclicks],false)) {
				//click done
				nclicks++;
			}
			vpDisplay::flush(*Ic);
		}
		vpDisplay::displayCross(*Ic,clicks[2],10,vpColor::red,3);
		vpDisplay::flush(*Ic);
	}

	std::cerr << "Starting ESMlib" << std::endl;
	double angle=atan2(clicks[2].get_i()-clicks[1].get_i(),clicks[2].get_j()-clicks[1].get_j());
	std::cerr << "angle is : " << angle << std::endl;

	this->posx=clicks[0].get_j();
	this->posy=clicks[0].get_i();
	this->sizx=vpImagePoint::distance(clicks[1],clicks[2]);
	this->sizy=vpImagePoint::distance(clicks[0],clicks[1]);
	this->alpha=angle;
}


void ESMTracking::setInitialEstimation(vpHomography &H) {
	T.homog[0]=H[0][0];
	T.homog[1]=H[0][1];
	T.homog[2]=H[0][2];
	T.homog[3]=H[1][0];
	T.homog[4]=H[1][1];
	T.homog[5]=H[1][2];
	T.homog[6]=H[2][0];
	T.homog[7]=H[2][1];
	T.homog[8]=H[2][2];
}

void ESMTracking::perceive() {
	if (color_input) vpImageConvert::convert(*Ic,*I);

    //copy I to Iesm 
    for (unsigned int i=0;i<I->getNumberOfPixel();i++)
        Iesm.data[i]=(float)I->bitmap[i];

    // Perform the tracking
    if (MakeTrack (&T, &Iesm)) {
      std::cerr << "ESMTracking::perceive() ERROR: Tracking failed" << std::endl;
      exit(0);
    }
    updateCornersFromHomography();
}

void ESMTracking::updateCornersFromHomography() { 
    //update template corners position
    float pIni[8];
    float *H=T.homog;
    for (int i=0; i<3; i++)
	for (int j=0; j<3; j++)
		tHc[i][j]=H[i*3+j];
    pIni[0] = 0;    pIni[1] = 0;      pIni[2] = sizx - 1; pIni[3] = 0;
    pIni[4] = sizx-1; pIni[5] = sizy - 1; pIni[6] = 0;      pIni[7] = sizy - 1;
    
    for (int i = 0; i < 4; i++) {
      pEnd[2*i] = (H[0]*pIni[2*i] + H[1]*pIni[2*i+1] + H[2])/
        (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
      pEnd[2*i+1] = (H[3]*pIni[2*i] + H[4]*pIni[2*i+1] + H[5])/
        (H[6]*pIni[2*i] + H[7]*pIni[2*i+1] + H[8]);
    }
}

void ESMTracking::draw(vpImage<vpRGBa> &Ic) {
    vpDisplay::displayLine (Ic, (int)pEnd[1], (int)pEnd[0], (int)pEnd[3], (int)pEnd[2],vpColor::green,3);
    vpDisplay::displayLine (Ic, (int)pEnd[3], (int)pEnd[2], (int)pEnd[5], (int)pEnd[4],vpColor::green,3);
    vpDisplay::displayLine (Ic, (int)pEnd[5], (int)pEnd[4], (int)pEnd[7], (int)pEnd[6],vpColor::green,3);
    vpDisplay::displayLine (Ic, (int)pEnd[7], (int)pEnd[6], (int)pEnd[1], (int)pEnd[0],vpColor::green,3);
}

void ESMTracking::draw(vpImage<unsigned char> &I) {
    vpDisplay::displayLine (I, (int)pEnd[1], (int)pEnd[0], (int)pEnd[3], (int)pEnd[2],vpColor::green,3);
    vpDisplay::displayLine (I, (int)pEnd[3], (int)pEnd[2], (int)pEnd[5], (int)pEnd[4],vpColor::green,3);
    vpDisplay::displayLine (I, (int)pEnd[5], (int)pEnd[4], (int)pEnd[7], (int)pEnd[6],vpColor::green,3);
    vpDisplay::displayLine (I, (int)pEnd[7], (int)pEnd[6], (int)pEnd[1], (int)pEnd[0],vpColor::green,3);
}


/** Map a point given in pixels wrt the template origin to the current position in image after the homography transform */
vpImagePoint ESMTracking::map(int u, int v) {
	float *H=T.homog;
	return vpImagePoint( (H[3]*u + H[4]*v + H[5]) / (H[6]*u + H[7]*v + H[8]),
			     (H[0]*u + H[1]*v + H[2]) / (H[6]*u + H[7]*v + H[8]));	
}

ESMTracking::~ESMTracking() {
	FreeTrack (&T);
}


ESMTrackingROSPublisher::ESMTrackingROSPublisher(ros::NodeHandle nh, std::string publish_topic, vpImage<unsigned char> *I, int posx, int posy, int sizx, int sizy, float alpha, int miter, int mprec):
	ESMTracking(I, posx, posy, sizx, sizy, alpha, miter, mprec),
	publish_topic_(publish_topic) {
	//Advertise object bounding box
	tt_pub_= nh.advertise<mar_msgs::TemplateTrack>(publish_topic_, 1);
}

ESMTrackingROSPublisher::ESMTrackingROSPublisher(ros::NodeHandle nh, std::string publish_topic, vpImage<vpRGBa> *Ic, int posx, int posy, int sizx, int sizy, float alpha, int miter, int mprec):
	ESMTracking(Ic, posx, posy, sizx, sizy, alpha, miter, mprec),
	publish_topic_(publish_topic) {
	//Advertise object bounding box
	tt_pub_= nh.advertise<mar_msgs::TemplateTrack>(publish_topic_, 1);
}

ESMTrackingROSSubscriber::ESMTrackingROSSubscriber(ros::NodeHandle nh, std::string tracker_topic): tracker_topic_(tracker_topic) {
		 //Subscribe to object bbox
		 ros::Subscriber bbox_sub_= nh.subscribe(tracker_topic, 1, &ESMTrackingROSSubscriber::messageReceivedCallback, this);
}
