/*
 * PCAutonomousGraspPlanning.h
 *
 *  Created on: 07/06/2012
 *      Author: mprats
 *      Mod: dfornas 23/01/2013
 */
#ifndef PCAUTONOMOUSGRASPPLANNING_H_
#define PCAUTONOMOUSGRASPPLANNING_H_

#include <mar_core/CPerception.h>
#include <mar_perception/VispUtils.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpPoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>

#include <tf/transform_datatypes.h>

#define DEFAULT_HAND_WIDTH	0.1
#define DEFAULT_GRASP_PENETRATION	0.05	//Maximum penetration of the hand around the object
#define DEFAULT_ALIGNED_GRASP false

typedef pcl::PointXYZRGB PointT;

/** Grasp planning taking as input a point cloud, a pair of grasp point in cloud coordinates. */
class PCAutonomousGraspPlanning : public CPerception {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  // Grasp 3D points
  vpPoint g1_, g2_;
  //Grasping params (to allow different grasps and radious (for grasp penetration)).
  double angle_, rad_, along_;

  //Now unused
  double hand_width_, grasp_penetration_;
  //Punto central del cilindro y la direccion.
  PointT axis_point_g; tf::Vector3 normal_g;
  bool aligned_grasp_;
  VispToTF vispToTF;  
  MarkerPublisher * cylPub;
  
 public:
  
  vpHomogeneousMatrix cMg, base_cMg; //< Grasp frame with respect to the camera after planning

  //With integuers to use trackbars
  int iangle, irad, ialong, ialigned_grasp;

  public:
  /** Constructor.
   * @param angle
   * @param rad
   * @param along
   * @param aligned_grasp
   * @param pointcloud A PCL point cloud pointer
   * */
  PCAutonomousGraspPlanning( double angle, double rad, double along, double aligned_grasp, pcl::PointCloud<PointT>::Ptr pointcloud): cloud_(pointcloud) {
    angle_=angle;iangle=angle*360.0/(2.0*M_PI);
    rad_=rad;irad=rad*100;
    along_=along;ialong=along*100;
    setHandWidth(DEFAULT_HAND_WIDTH);
    setAlignedGrasp(aligned_grasp);ialigned_grasp=aligned_grasp?1:0;
    setGraspPenetration(DEFAULT_GRASP_PENETRATION);
    vispToTF.addTransform(cMg, "/stereo", "/base_cMg", "1");
    vispToTF.addTransform(cMg, "/stereo", "/repositioned_cMg", "2");
  }

  /** Main function where segmentation is done */
  void perceive();

  /** Set the dimensions of the gripper (in meters) */
  void setHandWidth(double w) {hand_width_=w;}

  /** Set whether to perform a grasp aligned with the cylinder axis or not **/
  void setAlignedGrasp(bool a) {aligned_grasp_=a;}

  /** Set the max penetration of the grasp around the object (in meters) */
  void setGraspPenetration(double p) {grasp_penetration_=p;}

  /** Get the grasp frame with respect to the camera frame */
  vpHomogeneousMatrix get_cMg() {return cMg;}

  /** Recalculate cMg with current parameters */
  void recalculate_cMg();	

  /** Get the grasp frame pose with respect to an arbitrary frame 'b', given as input relative to the camera frame
   * @param bMc an homogeneous matrix with the camera frame given wrt the frame 'b'
   * @returns an homogeneous matrix with the grasp frame given wrt the frame 'b'
   */
  vpHomogeneousMatrix get_bMg(vpHomogeneousMatrix bMc) {return bMc*cMg;}

  ~PCAutonomousGraspPlanning() {}

  private:

  /** Comparer used in the sort function */
  bool sortFunction(const PointT& d1, const PointT& d2);

  /** Get the grasp frame with respect to the camera frame */
  void getMinMax3DAlongAxis(const pcl::PointCloud<PointT>::ConstPtr& cloud, PointT * max_pt, PointT * min_pt, PointT axis_point, tf::Vector3 * normal);

  /** Configure the camera based in int slider parameters */
  void intToConfig();

};

#endif /* PCAUTONOMOUSGRASPPLANNING_H_ */
