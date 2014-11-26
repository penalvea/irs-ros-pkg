/*
 * LaserPeakDetector.cpp
 *
 * Created on: 24/05/2012
 * Author: mprats
 */


#include <mar_perception/LaserPeakDetector.h>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <string>



double LaserPeakDetector::getIntersectionPoint(vpHomogeneousMatrix bMc, vpHomogeneousMatrix bMl, int col, double radius){

	//  Parametric equation of laser plane

	vpColVector normal_laser(3), point_laser(3);
	// X axis is the laser normal
	normal_laser[0]=bMl[0][0];
	normal_laser[1]=bMl[1][0];
	normal_laser[2]=bMl[2][0];
	point_laser[0]=bMl[0][3];
	point_laser[1]=bMl[1][3];
	point_laser[2]=bMl[2][3];

	//std::cout<<"Laser "<<normal_laser<<std::endl<<point_laser<<std::endl;


	//Points intersection between two rays from the camera and the laser plane

	vpColVector point_column1(3), point_column2(3);
	point_column1[0]=(col-grabber_->K.get_u0())/grabber_->K.get_px();
	point_column1[1]=(0-grabber_->K.get_v0())/grabber_->K.get_py();
	point_column1[2]=1;

	point_column2[0]=(col-grabber_->K.get_u0())/grabber_->K.get_px();
	point_column2[1]=((grabber_->image.getRows()-1)-grabber_->K.get_v0())/grabber_->K.get_py();
	point_column2[2]=1;

	//Changing axes
	vpHomogeneousMatrix cMp1(point_column1[0], point_column1[1], point_column1[2],0,0,0);
	vpHomogeneousMatrix cMp2(point_column2[0], point_column2[1], point_column2[2],0,0,0);
	vpHomogeneousMatrix bMp1=bMc*cMp1;
	vpHomogeneousMatrix bMp2=bMc*cMp2;

	//std::cout<<"Points "<<bMp1<<std::endl<<bMp2<<std::endl;

	//Plane: (p-p0)·n=0
	//Line: p=d*l+l0

	//Intersection: d=(p0-l0)·n /  l·n

	vpColVector l1(3);
	l1[0]=bMp1[0][3]-bMc[0][3];
	l1[1]=bMp1[1][3]-bMc[1][3];
	l1[2]=bMp1[2][3]-bMc[2][3];


	vpColVector l2(3);
	l2[0]=bMp2[0][3]-bMc[0][3];
	l2[1]=bMp2[1][3]-bMc[1][3];
	l2[2]=bMp2[2][3]-bMc[2][3];


	vpColVector point_cam(3);
	point_cam[0]=bMp1[0][3];
	point_cam[1]=bMp1[1][3];
	point_cam[2]=bMp1[2][3];

	float d1=vpColVector::dotProd((point_laser-point_cam),normal_laser)/vpColVector::dotProd(l1,normal_laser);

	vpColVector intersection_point1(3);
	intersection_point1[0]=bMp1[0][3]+l1[0]*d1;
	intersection_point1[1]=bMp1[1][3]+l1[1]*d1;
	intersection_point1[2]=bMp1[2][3]+l1[2]*d1;


	vpColVector point_cam2(3);
	point_cam2[0]=bMp2[0][3];
	point_cam2[1]=bMp2[1][3];
	point_cam2[2]=bMp2[2][3];

	float d2=vpColVector::dotProd((point_laser-point_cam2),normal_laser)/vpColVector::dotProd(l2,normal_laser);

	vpColVector intersection_point2(3);
	intersection_point2[0]=bMp2[0][3]+l2[0]*d2;
	intersection_point2[1]=bMp2[1][3]+l2[1]*d2;
	intersection_point2[2]=bMp2[2][3]+l2[2]*d2;

	//std::cout<<"Intersection Points "<<intersection_point1<<std::endl<<intersection_point2<<std::endl;


	// Intersection between line and sphere

	vpColVector sphere_centre(3);
	sphere_centre=0;

	double a, b, c;
	a=std::pow((intersection_point2[0]-intersection_point1[0]), 2.0)+std::pow((intersection_point2[1]-intersection_point1[1]), 2.0)+std::pow((intersection_point2[2]-intersection_point1[2]), 2.0);
	b=2*(((intersection_point2[0]-intersection_point1[0])*(intersection_point1[0]-sphere_centre[0]))+((intersection_point2[1]-intersection_point1[1])*(intersection_point1[1]-sphere_centre[1]))+((intersection_point2[2]-intersection_point1[2])*(intersection_point1[2]-sphere_centre[2])));
	c=std::pow((intersection_point1[0]-sphere_centre[0]),2.0)+std::pow((intersection_point1[1]-sphere_centre[1]),2.0)+std::pow((intersection_point1[2]-sphere_centre[2]),2.0)-std::pow(radius, 2.0);

	double delta=std::pow(b,2.0)-(4*a*c);

	/*std::cout<<"a="<<a<<std::endl;
	std::cout<<"b="<<b<<std::endl;
	std::cout<<"c="<<c<<std::endl;
	std::cout<<"delta="<<delta<<std::endl;*/

	if(delta<0){
		//std::cout<<"There is no intersection between the laser plane, the camera plane and the WS sphere"<<std::endl;
	}
	else if(delta==0){
		double d1=(-b)/(2*a);
		vpColVector final_point1(3), final_point2(3), final_point3(3);
		final_point1=intersection_point1+d1*(intersection_point2-intersection_point1);
		vpHomogeneousMatrix bMpoint1(final_point1[0],final_point1[1],final_point1[2],0,0,0);
		vpHomogeneousMatrix cMpoint1=bMc.inverse()*bMpoint1;
		if(cMpoint1[2][3]>0){
			return (cMpoint1[1][3]*grabber_->K.get_py()/cMpoint1[2][3])+grabber_->K.get_v0();
		}

		//std::cout<<"There is only a single intersection between the laser plane, the camera plane and the WS sphere"<<std::endl;
	}
	else{
		double d1=(-b-std::sqrt(delta))/(2*a);
		double d2=(-b+std::sqrt(delta))/(2*a);

		vpColVector final_point1(3), final_point2(3), final_point3(3);
		final_point1=intersection_point1+d1*(intersection_point2-intersection_point1);
		final_point2=intersection_point1+d2*(intersection_point2-intersection_point1);


		vpHomogeneousMatrix bMpoint1(final_point1[0],final_point1[1],final_point1[2],0,0,0), bMpoint2(final_point2[0],final_point2[1],final_point2[2],0,0,0);
		vpHomogeneousMatrix cMpoint1=bMc.inverse()*bMpoint1;
		vpHomogeneousMatrix cMpoint2=bMc.inverse()*bMpoint2;

		if(cMpoint1[2][3]>0){
			return (cMpoint1[1][3]*grabber_->K.get_py()/cMpoint1[2][3])+grabber_->K.get_v0();
		}
		else if(cMpoint2[2][3]>0){
			return (cMpoint2[1][3]*grabber_->K.get_py()/cMpoint2[2][3])+grabber_->K.get_v0();

		}
	}
	return -1;
	//std::cout<<"--------------------------------------------------------------------------------------------------"<<std::endl;
	//return intersection_point;

}

void LaserPeakDetector::setLimits(vpHomogeneousMatrix bMc, vpHomogeneousMatrix bMl){
	if(min_radius_!=-1 && max_radius_!=-1){
		double left_min=getIntersectionPoint(bMc, bMl, 0, min_radius_);
		double right_min=getIntersectionPoint(bMc, bMl, grabber_->image.getCols(), min_radius_);
		double left_max=getIntersectionPoint(bMc, bMl, 0, max_radius_);
		double right_max=getIntersectionPoint(bMc, bMl, grabber_->image.getCols(), max_radius_);


		if(left_min!=-1 && right_min!=-1){
			limits_[1]=left_min>right_min?(int)left_min+1:(int)right_min+1;
			if(limits_[1]<0){
				limits_[1]=0;
			}
			if(limits_[1]>grabber_->image.getRows()-1){
				limits_[1]=grabber_->image.getRows()-1;
			}

		}
		if(left_max!=-1 && right_max!=-1){
			limits_[0]=left_max<right_max?((int)left_max):((int)right_max);
			if(limits_[0]<0){
				limits_[0]=0;
			}
			if(limits_[0]>grabber_->image.getRows()-1){
				limits_[0]=grabber_->image.getRows()-1;
			}

		}
		if(limits_[0]>limits_[1]){
			int limit_aux=limits_[0];
			limits_[0]=limits_[1];
			limits_[1]=limit_aux;
		}

	}
}

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



void LastImageSubPixelLaserPeakDetector::perceive() {
	points.clear();
	cv::Mat restaNB,restaBN;
	cv::Mat Ig;
	vpImageConvert::convert(grabber_->image,Ig);
	restaNB = Ig - lastImage_; /// N : Now B : Before
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




HSVSubPixelLaserPeakDetector::HSVSubPixelLaserPeakDetector(VirtualImagePtr grabber):LaserPeakDetector(grabber){
	//Values for real
		iLowH = 15;
		iHighH = 91;

		iLowS = 0;
		iHighS = 90;

		iLowV = 181;
		iHighV = 255;


}

void HSVSubPixelLaserPeakDetector::perceive(){
	points.clear();
	cv::Mat img, filtered, filtered_aux, hsv, thresholding;
	vpImage<vpRGBa> Ic;
	Ic=grabber_->image;



	cv::namedWindow("Control", CV_WINDOW_NORMAL );
	cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
	cv::waitKey(5);

	vpImageConvert::convert(Ic, img);
	cv::GaussianBlur(img, filtered, cv::Size(5,5), 10, 10, 4);
	cv::cvtColor(filtered, filtered_aux, CV_RGBA2RGB);
	cv::cvtColor(filtered_aux, hsv,CV_RGB2HSV);

	cv::inRange(hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), thresholding); //Threshold the image
	cv::erode(thresholding, thresholding, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::dilate(thresholding, thresholding, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	cv::dilate(thresholding, thresholding, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::erode(thresholding, thresholding, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	//cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
	//cv::imshow("Display window", thresholding);

	//Laser detection
	for(int j=0; j<thresholding.cols; j++){
		int up=-1, down=-1;
		for(int i=limits_[0]; i<=limits_[1]; i++){
			if(thresholding.at<unsigned char>(i,j)!=0){
				down=i;
				if(up==-1)
					up=i;
			}

		}
		//The laser peak is between up and down
		//The brigthest pixel is selected
		if(up!=-1){
			int max=0;
			int row;
			for(int k=up; k<=down; k++){
				if(Ic[k][j].R+Ic[k][j].G+Ic[k][j].B>max){
					max=Ic[k][j].R+Ic[k][j].G+Ic[k][j].B;
					row=k;
				}


			}
			//Select the pixel with subpixel accuracy using the center of mass
			float mass=0;
			float suma=0;
			for(int k=row-5; k<=row+5; k++){
				if(k>=limits_[0] && k<=limits_[1]){
					suma+=(Ic[k][j].R+Ic[k][j].G+Ic[k][j].B)*k;
					mass+=Ic[k][j].R+Ic[k][j].G+Ic[k][j].B;
					//suma+=(Ic[k][j].G)*k;
					//mass+=Ic[k][j].G;
				}
			}
			vpColVector point(2);
			point[0]=suma/mass;
			point[1]=j;
			points.push_back(point);
		}
	}
}





SimulationLaserPeakDetector::SimulationLaserPeakDetector(VirtualImagePtr grabber):LaserPeakDetector(grabber){


		//Values for simulation blackbox

		iLowH = 33;
		iHighH = 118;

		iLowS = 0;
		iHighS = 255;

		iLowV = 40;
		iHighV = 255;
}



void SimulationLaserPeakDetector::perceive(){
	points.clear();
	if(limits_[0]==limits_[1]){
		return;
	}
	cv::Mat first_img, img, filtered, filtered_aux, hsv, thresholding;
	vpImage<vpRGBa> Ic;
	Ic=grabber_->image;


	/*cv::namedWindow("Control", CV_WINDOW_NORMAL );
	cvCreateTrackbar("LowH", "Control", &iLowH, 255); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
	cv::waitKey(5);*/
	vpImageConvert::convert(Ic, first_img);
	img=first_img(cv::Rect(0,limits_[0], Ic.getCols(), limits_[1]-limits_[0]));
	cv::GaussianBlur(img, filtered, cv::Size(5,5), 10, 10, 4);
	cv::cvtColor(filtered, filtered_aux, CV_RGBA2RGB);
	cv::cvtColor(filtered_aux, hsv,CV_RGB2HSV);

	cv::inRange(hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), thresholding); //Threshold the image
	cv::erode(thresholding, thresholding, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::dilate(thresholding, thresholding, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::dilate(thresholding, thresholding, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::erode(thresholding, thresholding, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	//cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
	//cv::imshow("Display window", thresholding);
	//Laser detection
	for(int j=0; j<thresholding.cols; j++){
			int up=-1, down=-1;
			for(int i=0; i<thresholding.rows; i++){
				if(thresholding.at<unsigned char>(i,j)!=0){
					if(up==-1)
						up=i;
					else
						down=i;
				}
				else{
					if(up!=-1){
						vpColVector point(2);
						point[0]=((up+down)/2)+limits_[0];
						point[1]=j;
						points.push_back(point);
						up=-1;
						down=-1;
					}
				}
			}
			if(up!=-1){
				vpColVector point(2);
				point[0]=((up+thresholding.rows-1)/2)+limits_[0];
				point[1]=j;
				points.push_back(point);
			}
		}



}







