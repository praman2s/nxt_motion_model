#ifndef MOTIONCAPTURE2D_H_
#define MOTIONCAPTURE2D_H_

// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
// OpenCV Includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>


// BOOST
#include <boost/units/systems/si.hpp>
#include <string>


class MotionCapture
{
public:
	
	MotionCapture();
        virtual ~MotionCapture();
        int MotionCapture2D( IplImage* input_image );

	

private:

	
	IplImage* LoadBackgroundImage();

	
	IplImage* RegionOfInterest( IplImage* input_image, double scale );

	/**
	 * This is a function that will take in an arbitrary number of images and create a display for
	 * them that will serve as the Heads Up Display (HUD) of the Visual Servoing Application.
	 * This is a modified version of the source code found here:
	 * http://opencv.willowgarage.com/wiki/DisplayManyImages
	 */
	void HUD(char* title, int nArgs, ...);

protected:
	
	ros::NodeHandle m_node_handler;
        ros::Time m_time_when_lost;
	IplImage* m_background_image;
	int m_image_height;
	int m_image_width;


};

#endif 
