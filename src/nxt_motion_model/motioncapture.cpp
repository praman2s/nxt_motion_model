/**
 * This is the main source code file for running the Visual Servoing application. This application 
 * is dependent on the Robot Operating System (ROS) [www.ros.org], and will not run unless the
 * required ROS components are installed. You can do this by running the "rosdep install" command
 * from the base folder.
 *
 * This file is provided as is without any form or warranty.
 *
 * Author: Matthew Roscoe (mat.roscoe@unb.ca)
 */

// ROS


// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>



#include "motioncapture.h"

namespace enc = sensor_msgs::image_encodings;

MotionCapture::MotionCapture()
{
	m_background_image = LoadBackgroundImage();

}

MotionCapture::~MotionCapture()
{
	
}

int
MotionCapture::MotionCapture2D( IplImage* input_image )
{
	bool return_val = 0;

	double x_offset = 0;
	double y_offset = 0;
	double rot_offset = 0;

	IplImage* cv_image  ;
	IplImage* blob_image;

	
	double  temp_tracked_blob_distance = 0;

	double maxx;
	double minx;
	double maxy;
	double miny;
	double temp_x;
	double temp_y;
	double dist_x;
	double dist_y;
	double distance;

	if( !input_image )
	{
		ROS_ERROR( "Error in input image!" );
		return false;
	}

	cv_image = input_image;

	m_image_height = cv_image->height;
	m_image_width = cv_image->width;

	
	IplImage* background_threshold = cvCreateImage( cvGetSize( m_background_image ), 8, 1 );
	cvCvtColor( m_background_image, background_threshold, CV_BGR2GRAY );
	cvSmooth( background_threshold, background_threshold, CV_GAUSSIAN, 11, 11 );
	

	IplImage* gray = cvCreateImage( cvGetSize( cv_image ), 8, 1 );
	cvCvtColor( cv_image, gray, CV_BGR2GRAY );
	cvSmooth( gray, gray, CV_GAUSSIAN, 11, 11 );

	

	cvShowImage( "GRAY", gray ); 



	return 0; 
}
IplImage*
MotionCapture::LoadBackgroundImage()
{
	IplImage* background_image;
	std::string mode;

	
	mode = "background.png";
	

	try
	{
	  std::string package_path = ros::package::getPath("nxt_motion_model") + "/data/" + mode;
	  std::cout << "Package Path:\t" << package_path.c_str() << std::endl;
	  background_image = cvLoadImage( package_path.c_str() );
	}
	catch ( cv::Exception& e )
	{
		ROS_ERROR( "Could not load background image" );
	}

	return background_image;
}

IplImage*
MotionCapture::RegionOfInterest( IplImage* input_image, double scale )
{
	if( scale <= 0 || scale >= 1 )
	{
		ROS_ERROR( "Invalid scale provided setting to 0.7" );
		scale = 0.7;
	}

	int width = (int)(input_image->width * scale );
	int height = (int)( input_image->height * scale );

	int x = ( (input_image->width - width) / 2 );
	int y = ( (input_image->height - height) / 2 );

	cvSetImageROI( input_image, cvRect( x, y, width, height ) );

	
	return input_image;
}



void
MotionCapture::HUD(char* title, int nArgs, ...) {

    // img - Used for getting the arguments
    IplImage *img;

    // DispImage - the image in which input images are to be copied
    IplImage *DispImage;

    int size;
    int i;
    int m, n;
    int x, y;

    // w - Maximum number of images in a row
    // h - Maximum number of images in a column
    int w, h;

    // scale - How much we have to resize the image
    float scale;
    int max;

    // If the number of arguments is lesser than 0 or greater than 12
    // return without displaying
    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 12) {
        printf("Number of arguments too large....\n");
        return;
    }
    // Determine the size of the image,
    // and the number of rows/cols
    // from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 300;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 350;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }

    // Create a new 3 channel image
    DispImage = cvCreateImage( cvSize( 50 + size*w, 60 + size*h), 8, 3 );

    // Used to get the arguments passed
    va_list args;
    va_start(args, nArgs);

    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

        // Get the Pointer to the IplImage
        img = va_arg(args, IplImage*);

        // Check whether it is NULL or not
        // If it is NULL, release the image, and return
        if(img == 0) {
            printf("Invalid arguments");
            cvReleaseImage(&DispImage);
            return;
        }

        // Find the width and height of the image
        x = img->width;
        y = img->height;

        // Find whether height or width is greater in order to resize the image
        max = (x > y)? x: y;

        // Find the scaling factor to resize the image
        scale = (float) ( (float) max / size );

        // Used to Align the images
        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size;
        }

        // Set the image ROI to display the current image
        cvSetImageROI(DispImage, cvRect(m, n, (int)( x/scale ), (int)( y/scale )));

        // Resize the input image and copy the it to the Single Big Image
        cvResize( img, DispImage );

        // Reset the ROI in order to display the next image
        cvResetImageROI(DispImage);
    }

    // Create a new window, and show the Single Big Image
    cvNamedWindow( title, 1 );
    cvShowImage( title, DispImage);

    //cvWaitKey();
    //cvDestroyWindow(title);

    // End the number of arguments
    va_end(args);

    // Release the Image Memory
    //cvReleaseImage(&DispImage);
}

void ImageCallback( const sensor_msgs::ImageConstPtr& image_message )
{
	cv_bridge::CvImagePtr cv_img_tmp;
  	IplImage ipl_img_tmp;
	IplImage *cv_image = NULL;

	try
	{
	    cv_img_tmp =  cv_bridge::toCvCopy(image_message, "bgr8");
	    ipl_img_tmp = cv_img_tmp->image;
	    ROS_INFO("Sucessfully converted");
		
	}
	catch( cv_bridge::Exception& e )
	{
		ROS_ERROR( "Could not convert from '%s' to 'bgr8'.", image_message->encoding.c_str() );
	}
	cv_image = &ipl_img_tmp;
	IplImage* gray = cvCreateImage( cvGetSize( cv_image ), 8, 1 );
	cvCvtColor( cv_image, gray, CV_BGR2GRAY );
	cvSmooth( gray, gray, CV_GAUSSIAN, 11, 11 );
	cvNamedWindow("stabilized image", CV_WINDOW_AUTOSIZE );
	IplImage* image = cvCreateImage(cvSize(900,650),IPL_DEPTH_8U,3);
	cvShowImage("stabilized image", cv_image);
	cvShowImage( "GRAY", gray ); 

	
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "motioncapture");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw", 1000, ImageCallback);
  //MotionCapture ic;
  ros::spin();
  return 0;
}
