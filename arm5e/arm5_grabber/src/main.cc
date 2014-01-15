/** iec61883 grabber from dvgrab code. Mario Prats (mprats@uji.es) */

#include "ieee1394io.h"
#include "raw1394util.h"
#include "dvframe.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
	

iec61883Reader *m_reader=NULL;
iec61883Connection* m_connection=NULL;
int m_channel=0;

#define FRAMESIZE	1244160		//720*576 pixels * 3 rgb bytes

void busResetCallback(void*) {
	if ( m_reader )
        {
                if ( ! m_reader->WaitForAction( 1 ) )
                {
                        std::cerr << "Error: timed out waiting for DV after bus reset" << std::endl; 
                } else
                // Otherwise, reestablish the connection
                if ( m_connection )
                {
                        int newChannel = m_connection->Reconnect();
                        if ( newChannel != m_channel )
                        {
                                std::cerr << "Error: unable to reestablish connection after bus reset"  << std::endl;
			}
		}
	}
}

int main(int argc, char **argv) {
	int m_node=-1;
	int m_port=-1;
	octlet_t m_guid=0;
	int m_buffers=100;

	//ROS stuff
	ros::init(argc,argv,"ARM5EGrabber");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher img_pub = it.advertise("/arm5e/camera", 1);
	ros::Publisher img_info_pub=nh.advertise<sensor_msgs::CameraInfo>("/arm5e/camera_info", 10);

	int subsample=1;
 	nh.param("subsample", subsample, subsample);

	//Discover the camera on the bus
	m_node = discoverAVC( &m_port, &m_guid );
	iec61883Connection::CheckConsistency( m_port, m_node );
	std::cerr << "m_port: " << m_port << " m_node: " << m_node << " m_guid: " << m_guid << std::endl;
	AVC *m_avc = new AVC( m_port );
        if ( ! m_avc )
       		std::cerr << "failed to initialize AV/C" << std::endl;

	//Establishing the connection with the camera
	m_connection = new iec61883Connection( m_port, m_node );
	if ( ! m_connection )  {
		std::cerr << "failed to establish isochronous connection" << std::endl;
		exit(0);
	}
	m_channel = m_connection->GetChannel();
	std::cerr << "Established connection over channel " <<  m_channel << std::endl;

	//frame reader
	m_reader = new iec61883Reader( m_port, m_channel, m_buffers, &busResetCallback);

	static Frame *m_frame;
	m_reader->StartThread();
//	Do we need to call Play on AVC??
//	m_avc->Play( m_port );
	char image_data[FRAMESIZE];
	sensor_msgs::Image img;
	sensor_msgs::CameraInfo img_info;
	img.encoding=std::string("rgb8");
	img.is_bigendian=0;
	img.data.resize((int)(FRAMESIZE/(float)subsample));
	img_info.distortion_model=std::string("plumb_bob");
	img_info.D.resize(5);
	img_info.D[0] = -0.36999;
	img_info.D[1] = 0.17395;
	img_info.D[2] = -0.00155;
	img_info.D[3] = 0.00287;
	img_info.D[4] = 0.0;

	img_info.K[0] = 626.07863/subsample;img_info.K[1] = 0;img_info.K[2] = 368.93077/subsample;
	img_info.K[3] = 0;img_info.K[4] = 682.77891/subsample;img_info.K[5] = 284.82611/subsample;
	img_info.K[6] = 0;img_info.K[7] = 0;img_info.K[8] = 1;

	img_info.R[0] = 1;img_info.R[1] = 0;img_info.R[2] = 0;
	img_info.R[3] = 0;img_info.R[4] = 1;img_info.R[5] = 0;
	img_info.R[6] = 0;img_info.R[7] = 0;img_info.R[8] = 1;

	img_info.P[0] = 626.07863/subsample;img_info.P[1] = 0;img_info.P[2] =  368.93077/subsample; img_info.P[3]=0;
	img_info.P[4] = 0;img_info.P[5] =  682.77891/subsample;img_info.P[6]=284.82611/subsample; img_info.P[7]=0;
	img_info.P[8] = 0;img_info.P[9] = 0;img_info.P[10] = 1; img_info.P[11]=0;
	//img_info.P[0] = 640.724172586983740/subsample;img_info.P[1] = 0;img_info.P[2] = 372.070928282283490/subsample;img_info.P[3] = 0;
	//img_info.P[4] = 0;img_info.P[5] = 697.195083256988260/subsample;img_info.P[6] = 284.162939041584420/subsample;img_info.P[7] = 0;
	//img_info.P[8] = 0;img_info.P[9] = 0;img_info.P[10] = 1;img_info.P[11] = 0;

	while (ros::ok()) {
		ros::spinOnce();

		//Getting the frame
		m_reader->WaitForAction( );
		if ( ( m_frame = m_reader->GetFrame() ) == NULL ) {
			std::cerr << "Error getting frame "  << std::endl;
                        ros::shutdown();
		} else {
			//Access to the RGB values and publish ROS image
			DVFrame *dvframe = (DVFrame*)m_frame;
			int width=dvframe->GetWidth();
			int height=dvframe->GetHeight();
			dvframe->ExtractRGB(image_data);

			//Publish image
			img_info.header.stamp=ros::Time::now();
			img.header.stamp=img_info.header.stamp;
		        img.height=(int)(height/(float)subsample);
		        img.width=(int)(width/(float)subsample);
		        img.step=img.width*3;
		        
			// Calibration parameters:
		        img_info.width=img.width;
		        img_info.height=img.height;
			
			//Copy frame data into ROS message
			if (subsample==1) 
				memcpy(&(img.data[0]),image_data,FRAMESIZE);
			else {
				int dest_count=0;
				int orig_count=0;
				for (int i=0; i<img_info.height; i++) {
				  for (int j=0; j<img_info.width; j++) {
					img.data[dest_count]=image_data[orig_count];
					img.data[dest_count+1]=image_data[orig_count+1];
					img.data[dest_count+2]=image_data[orig_count+2];
				  	dest_count+=3;
					orig_count+=3*subsample;
				  }
				  orig_count+=img_info.width*subsample*3;
				}
			}
			//publish image and info
			img_pub.publish(img);
		        img_info_pub.publish(img_info);

			m_reader->DoneWithFrame( m_frame );
		}
	}
	m_reader->StopThread();
	
	return 0;
}
