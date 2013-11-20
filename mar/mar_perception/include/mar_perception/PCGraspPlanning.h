/*
 * PCGraspPlanning.h
 *
 *  Created on: 07/06/2012
 *      Author: mprats
 */

#ifndef PCGRASPPLANNING_H_
#define PCGRASPPLANNING_H_

#include <mar_core/CPerception.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpImagePoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>

#define DEFAULT_HAND_WIDTH	0.1
#define DEFAULT_GRASP_PENETRATION	0.05	//Maximum penetration of the hand around the object


/** Grasp planning taking as input a point cloud, an image taken from the origin of the point cloud, and a pair of grasp point in image coordinates.
 * The pair of grasp points specified with respect to the image origin are back-projected into the point cloud, and a suitable grasp is computed in 3D around that region
 */
class PCGraspPlanning : public CPerception {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
	vpImage<vpRGBa> I_;
	vpImagePoint g1_, g2_;
	vpCameraParameters k_;

	double hand_width_, grasp_penetration_;
public:
	vpHomogeneousMatrix cMg;	///< Grasp frame with respect to the camera after planning

public:
	/** Constructor.
	 * @param I an image taken from the origin of the point cloud
	 * @param g1,g2 a pair of grasp points in image coordinates
	 * @param pointcloud A PCL point cloud pointer
	 * */
	PCGraspPlanning(vpImage<vpRGBa> &I, vpImagePoint &g1, vpImagePoint &g2, \
			vpCameraParameters &k, \
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud): cloud_(pointcloud) {
		I_=I;
		g1_=g1;
		g2_=g2;
		k_=k;
		setHandWidth(DEFAULT_HAND_WIDTH);
		setGraspPenetration(DEFAULT_GRASP_PENETRATION);
	}

	void perceive();

	/** Set the dimensions of the gripper (in meters) */
	void setHandWidth(double w) {hand_width_=w;}

	/** Set the max penetration of the grasp around the object (in meters) */
	void setGraspPenetration(double p) {grasp_penetration_=p;}

	/** Get the grasp frame with respect to the camera frame */
	vpHomogeneousMatrix get_cMg() {return cMg;}

	/** Get the grasp frame pose with respect to an arbitrary frame 'b', given as input relative to the camera frame
	 * @param bMc an homogeneous matrix with the camera frame given wrt the frame 'b'
	 * @returns an homogeneous matrix with the grasp frame given wrt the frame 'b'
	 */
	vpHomogeneousMatrix get_bMg(vpHomogeneousMatrix bMc) {return bMc*cMg;}

	~PCGraspPlanning() {}
};

#endif /* PCGRASPPLANNING_H_ */
