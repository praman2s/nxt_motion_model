#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>


namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "RAW IMAGE";
static const char W1[] = "HSV IMAGE";
static const char W2[] = "LED";
static const char W3[] = "BLOB";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("rawimage", 1);
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);

    cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow(WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    image_pub_.publish(cv_ptr->toImageMsg());
    //Write your logic here or try to call your logic from here
    channelizingCb(cv_ptr);

    
  }
 
  void channelizingCb(cv_bridge::CvImagePtr cv_ptr)
  {
	cv::Mat hsv;
        IplImage* frame=0;
	IplImage buf;
        IplImage *src_img;
        buf = cv_ptr->image;
    	src_img = &buf;
        double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
  	cv::Point matchLoc;
	frame=cvCloneImage(src_img); 
	cvSmooth(frame, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel
        IplImage* gray = cvCreateImage(cvGetSize(frame), 8, 1);
	cvCvtColor(frame, gray, CV_BGR2GRAY); //Change the color format from BGR to HSV
	cv::Mat image(gray);
	minMaxLoc(image, &minVal, &maxVal, &minLoc, &maxLoc);
        double thresh = maxVal*0.8; // 0.8 is a magic number. Need to change based on histogram
	IplImage* thres_image = cvCreateImage(cvGetSize(frame), 8, 1);
        cv::Mat dest(thres_image);
	cv::waitKey(3);
        cv::threshold(image,dest,thresh, 255,CV_THRESH_BINARY);
        cv::imshow(W2,dest);
 	cv::waitKey(3);
        cvReleaseImage(&gray);            
        cvReleaseImage(&thres_image);
	cv::waitKey(20);
  }

IplImage* GetThresholdedImage(IplImage* imgHSV){       
       IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
       cvInRangeS(imgHSV, cvScalar(170,160,60), cvScalar(180,256,256), imgThresh);
       return imgThresh;
} 


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
