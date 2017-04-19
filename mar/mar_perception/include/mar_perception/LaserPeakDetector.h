#ifndef LASERPEAKDETECTOR_H_
#define LASERPEAKDETECTOR_H_

//#include <opencv2/imgproc/imgproc.hpp>

#include <visp/vpImageConvert.h>
#include <visp/vpImage.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>

#include <mar_core/CPerception.h>
#include <mar_perception/VirtualImage.h>
#include <mar_robot_arm5e/ARM5Arm.h>

#include <vector>

#include <boost/shared_ptr.hpp>
#include <cv.h>

/** Abstract class for laser peak detectors */
class LaserPeakDetector : public CPerception
{
protected:
  VirtualImagePtr grabber_;	///< Pointer to the image grabber
  double min_radius_, max_radius_;
  std::vector<int> limits_;
public:
  std::vector<vpColVector> points;	///< 2D points detected as peaks

  LaserPeakDetector(VirtualImagePtr grabber)
  {
    grabber_ = grabber;
    min_radius_ = -1;
    max_radius_ = -1;
    limits_.push_back(0);
    limits_.push_back(grabber_->image.getRows() - 1);
  }

  void setRadiusMin(double radius)
  {
    min_radius_ = radius;
  }
  void setRadiusMax(double radius)
  {
    max_radius_ = radius;
  }
  void setLimits(vpHomogeneousMatrix bMc, vpHomogeneousMatrix bMl);
  virtual void perceive()=0;

  std::vector<int> getLimits()
  {
    return limits_;
  }
  std::vector<vpColVector> &getPeaks()
  {
    return points;
  }
  VirtualImagePtr getGrabber()
  {
    return grabber_;
  }

  virtual ~LaserPeakDetector()
  {
  }

private:
  double getIntersectionPoint(vpHomogeneousMatrix bMc, vpHomogeneousMatrix bMl, int col, double radius);

};
typedef boost::shared_ptr<LaserPeakDetector> LaserPeakDetectorPtr;

/** Simple laser peak detector based on color segmentation
 */
class SimpleLaserPeakDetector : public LaserPeakDetector
{
  vpRGBa reference_color_;
  double tolerance_;

public:
  SimpleLaserPeakDetector(VirtualImagePtr grabber, vpRGBa reference_color = vpRGBa(200, 255, 200),
                          double tolerance = 30) :
      LaserPeakDetector(grabber), reference_color_(reference_color), tolerance_(tolerance)
  {
  }

  virtual void perceive();

  ~SimpleLaserPeakDetector()
  {
  }
};
typedef boost::shared_ptr<SimpleLaserPeakDetector> SimpleLaserPeakDetectorPtr;

/** Simple laser peak detector based on color segmentation. Subpixel accuracy
 */
class SimpleSubPixelLaserPeakDetector : public LaserPeakDetector
{
  vpRGBa reference_color_;
  double tolerance_;

public:
  SimpleSubPixelLaserPeakDetector(VirtualImagePtr grabber, vpRGBa reference_color = vpRGBa(200, 200, 200),
                                  double tolerance = 100) :
      LaserPeakDetector(grabber), reference_color_(reference_color), tolerance_(tolerance)
  {
  }

  virtual void perceive();

  ~SimpleSubPixelLaserPeakDetector()
  {
  }
};
typedef boost::shared_ptr<SimpleSubPixelLaserPeakDetector> SimpleSubPixelLaserPeakDetectorPtr;

/** Laser peak detector based on last image. Subpixel accuracy
 */
class LastImageSubPixelLaserPeakDetector : public LaserPeakDetector
{
  cv::Mat lastImage_;
  int distance_threshold_;
  int value_threshold_;

public:
  LastImageSubPixelLaserPeakDetector(VirtualImagePtr grabber) :
      LaserPeakDetector(grabber)
  {
    distance_threshold_ = 10; //units are pixels
    value_threshold_ = 35; // 0-254
    cv::Mat Ig;
    vpImageConvert::convert(grabber_->image, Ig);
    lastImage_ = Ig.clone();
  }

  virtual void perceive();

  ~LastImageSubPixelLaserPeakDetector()
  {
  }
};
typedef boost::shared_ptr<LastImageSubPixelLaserPeakDetector> LastImageSubPixelLaserPeakDetectorPtr;

/** Laser peak detector based on huge, saturation and value segmentation. Subpixel accuray
 */
class HSVSubPixelLaserPeakDetector : public LaserPeakDetector
{
  //float r_max_, r_min_;
  int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

public:

  HSVSubPixelLaserPeakDetector(VirtualImagePtr grabber);

  virtual void perceive();

  ~HSVSubPixelLaserPeakDetector()
  {
  }
};
typedef boost::shared_ptr<HSVSubPixelLaserPeakDetector> HSVSubPixelLaserPeakDetectorPtr;

/** Laser peak detector based on huge, saturation and value segmentation. For simulation
 */
class SimulationLaserPeakDetector : public LaserPeakDetector
{
 // float r_max_, r_min_;
  int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

public:

  SimulationLaserPeakDetector(VirtualImagePtr grabber);

  virtual void perceive();

  ~SimulationLaserPeakDetector()
  {
  }
};
typedef boost::shared_ptr<SimulationLaserPeakDetector> SimulationLaserPeakDetectorPtr;

#endif
