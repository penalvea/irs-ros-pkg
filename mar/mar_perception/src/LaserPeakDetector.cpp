/*
 * LaserPeakDetector.cpp
 *
 *  Created on: 24/05/2012
 *      Author: mprats
 */


#include <mar_perception/LaserPeakDetector.h>
#include <stdlib.h>

void SimpleLaserPeakDetector::perceive() {
	//For each two columns, search the peak according to the RGB color (most similar to reference_color_)
	points.clear();
	for (unsigned int c=0; c<grabber_->image.getCols(); c++) {
		int similarity=1000;
		int mr=0,mc=0;
		int localsim;
		for (unsigned int r=0; r<grabber_->image.getRows() ; r++) {
			localsim=abs(grabber_->image(r,c).R-reference_color_.R) + abs(grabber_->image(r,c).G-reference_color_.G) + abs(grabber_->image(r,c).B-reference_color_.B);
			if (localsim<similarity) {
				similarity=localsim;
				mr=r; mc=c;
			}
		}
		if (similarity<tolerance_) {
			vpColVector point(2);
			point[0]=mr; point[1]=mc;
			points.push_back(point);
		}
	}
}

void SimpleSubPixelLaserPeakDetector::perceive() {
	//For each two columns, search the peak according to the RGB color (most similar to reference_color_) in subpixel accuracy
	points.clear();
	for (unsigned int c=0; c<grabber_->image.getCols(); c++) {
		double similarity=0;
		double mr=0,mc=0,nmatch=0;
		int localsim;
		for (unsigned int r=0; r<grabber_->image.getRows() ; r++) {
			localsim=abs(grabber_->image(r,c).R-reference_color_.R) + abs(grabber_->image(r,c).G-reference_color_.G) + abs(grabber_->image(r,c).B-reference_color_.B);
			if (localsim<tolerance_) {
				similarity+=localsim;
				mr+=r; mc+=c;
				nmatch++;
			}
		}
		if (nmatch>0) {
			similarity/=nmatch;
			mr/=nmatch;
			mc/=nmatch;
			vpColVector point(2);
			point[0]=mr; point[1]=mc;
			points.push_back(point);
		}
	}
}

/*
std::vector<vpColVector> LaserPeakDetection::detect2(vpImage<vpRGBa> &I) {
	//For each columns, search the peak on the G channel
	std::vector<vpColVector> points_vector;
	for (unsigned int c=0; c<I.getCols(); c++) {
		int mr, n;
		double avgr=0;
		mr=0; n=0;
		for (unsigned int r=0; r<I.getRows() ; r++) {
			if (I(r,c).G>245) {
				mr+=r; n++;
			}
			avgr=(double)mr/(double)n;
		}
		if (avgr>0) {
			vpColVector point(2);
			point[0]=avgr; point[1]=c;
			points_vector.push_back(point);
		}
	}
	return points_vector;

}



std::vector<vpColVector> LaserPeakDetection::detect4(vpImage<vpRGBa> &I) {
	//Search only from the latest point found to a threshold over the highest found
	std::vector<vpColVector> points_vector;
	if (last==NULL) {
		last=new double[I.getCols()];
		for (unsigned int c=0; c<I.getCols(); c++) last[c]=I.getRows()-1;
		highest=I.getRows()-1;
	}

	for (unsigned int c=0; c<I.getCols(); c++) {
		double similarity=0;
		double mr=0,mc=0,nmatch=0;
		int localsim;
		for (int r=last[c]; r>highest-100; r--) {
		  if (r>0) {
			localsim=abs(I(r,c).R-200) + abs(I(r,c).G-255) + abs(I(r,c).B-200);
			if (localsim<40) {
				similarity+=localsim;
				mr+=r; mc+=c;
				nmatch++;
			}
		  }
		}
		if (nmatch>0) {
			similarity/=nmatch;
			mr/=nmatch;
			mc/=nmatch;
			vpColVector point(2);
			point[0]=mr; point[1]=mc;
			points_vector.push_back(point);
			last[c]=mr;
			(mr<highest)? highest=mr : true;
			std::cerr << "highest is " << highest << std::endl;
		}
	}
	return points_vector;
}
 */

#include <opencv2/highgui/highgui.hpp>

void MiquelSubPixelLaserPeakDetector::perceive() {
  points.clear();
  cv::Mat restaNB,restaBN;
  cv::Mat Ig;
  vpImageConvert::convert(grabber_->image,Ig);
  restaNB = Ig - lastImage_; /// N : Now   B : Before
  restaBN = - Ig + lastImage_;
  //es mejor realizar la resta en BGR y después pasar a grises que al revés
  cv::cvtColor(restaNB,restaNB,CV_BGR2GRAY);
  cv::cvtColor(restaBN,restaBN,CV_BGR2GRAY);
  cv::Mat show_image = Ig.clone(); //imagen para marcar los puntos del laser

  for (unsigned int c=0; c<grabber_->image.getCols(); c++) {
    float maxNB = -1;
    float maxNB_r = -1;
    float maxBN = -1;
    float maxBN_r = -1;
    for (unsigned int r=0; r<grabber_->image.getRows() ; r++) {
      if(restaNB.at<uchar>(r,c)>maxNB)
      {
        maxNB = restaNB.at<uchar>(r,c);
        maxNB_r=r;
      }
      if(restaBN.at<uchar>(r,c)>maxBN)
      {
        maxBN = restaBN.at<uchar>(r,c);
        maxBN_r=r;
      }
    }
    int distance = abs(maxBN_r-maxNB_r);
    if((maxNB_r!=-1)&&(maxNB > value_threshold_)&&(distance < distance_threshold_)){
      vpColVector point(2);
      point[0]=(maxBN_r+maxNB_r)/2; point[1]=c;
      points.push_back(point);
      cv::circle(show_image,cv::Point(point[1],point[0]),2,cv::Scalar(0,0,0));
    }
  }
  //para jugar con la calibración
  cv::namedWindow("Calibration", CV_WINDOW_NORMAL );
  cv::createTrackbar( "Distance", "Calibration", &distance_threshold_, 255 );
  cv::createTrackbar( "Threshold", "Calibration", &value_threshold_, 255 );
  cv::imshow("Calibration", show_image );
  cv::waitKey(5);
  lastImage_ = Ig.clone();

}
