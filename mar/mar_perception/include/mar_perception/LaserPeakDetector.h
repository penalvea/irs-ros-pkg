#include <visp/vpImage.h>
#include <visp/vpColVector.h>

#include <mar_core/CPerception.h>
#include <mar_perception/VirtualImage.h>

#include <vector>

#include <boost/shared_ptr.hpp>
#include <cv.h>

/** Abstract class for laser peak detectors */
class LaserPeakDetector: public CPerception {
protected:
	VirtualImagePtr grabber_;	///< Pointer to the image grabber
public:
	std::vector<vpColVector> points;	///< 2D points detected as peaks

public:
	LaserPeakDetector(VirtualImagePtr grabber) {grabber_=grabber;}

	virtual void perceive()=0;

	std::vector<vpColVector> &getPeaks() {return points;}

	virtual ~LaserPeakDetector() {}
};
typedef boost::shared_ptr<LaserPeakDetector> LaserPeakDetectorPtr;

/** Simple laser peak detector based on color segmentation
 */
class SimpleLaserPeakDetector: public LaserPeakDetector {
	vpRGBa reference_color_;
	double tolerance_;

public:
	SimpleLaserPeakDetector(VirtualImagePtr grabber, vpRGBa reference_color=vpRGBa(200,255,200), double tolerance=30):
		LaserPeakDetector(grabber), reference_color_(reference_color), tolerance_(tolerance) {}

	virtual void perceive();

	~SimpleLaserPeakDetector() {}
};
typedef boost::shared_ptr<SimpleLaserPeakDetector> SimpleLaserPeakDetectorPtr;

/** Simple laser peak detector based on color segmentation. Subpixel accuracy
 */
class SimpleSubPixelLaserPeakDetector: public LaserPeakDetector {
	vpRGBa reference_color_;
	double tolerance_;

public:
	SimpleSubPixelLaserPeakDetector(VirtualImagePtr grabber, vpRGBa reference_color=vpRGBa(200,255,200), double tolerance=30):
		LaserPeakDetector(grabber), reference_color_(reference_color), tolerance_(tolerance) {}

	virtual void perceive();

	~SimpleSubPixelLaserPeakDetector() {}
};
typedef boost::shared_ptr<SimpleSubPixelLaserPeakDetector> SimpleSubPixelLaserPeakDetectorPtr;


#include <opencv2/imgproc/imgproc.hpp>
#include <visp/vpImageConvert.h>

/** Miquel laser peak detector based on color segmentation. Subpixel accuracy
 */
class MiquelSubPixelLaserPeakDetector: public LaserPeakDetector {
        cv::Mat lastImage_;
        int distance_threshold_;
        int value_threshold_;

public:
        MiquelSubPixelLaserPeakDetector(VirtualImagePtr grabber):LaserPeakDetector(grabber) {
          distance_threshold_ = 10; //units are pixels
          value_threshold_ = 35;    // 0-254
          cv::Mat Ig;
          vpImageConvert::convert(grabber_->image,Ig);
          lastImage_ = Ig.clone();
        }

        virtual void perceive();

        ~MiquelSubPixelLaserPeakDetector() {}
};
typedef boost::shared_ptr<MiquelSubPixelLaserPeakDetector> MiquelSubPixelLaserPeakDetectorPtr;
