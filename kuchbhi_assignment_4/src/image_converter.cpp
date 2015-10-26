#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>
//#include "opencv2/imgcodecs/imgcodecs.hpp"

using namespace cv;


static const std::string OPENCV_WINDOW = "Background Subtraction Output";
cv::BackgroundSubtractorMOG2 bg(10, 13, true);
//BackgroundSubtractorMOG2 pMOG2; //MOG2 Background subtractor
//using namespace cv;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
	int color_mode_;  
	cv::Mat prev_, next_, bgimage;
  
public:
  ImageConverter(int color_mode_)
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
		this->color_mode_ = color_mode_;
    cv::namedWindow(OPENCV_WINDOW);

    //pMOG2(5, 10, true); //MOG2 approach, 
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

		next_ = cv_ptr->image;
/*    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(next_, cv::Point(50, 50), 10, CV_RGB(255,0,0));
*/
    // Update GUI Window
		if(color_mode_)
			cv::cvtColor(next_, next_, CV_RGB2GRAY);


    bg(next_, bgimage, -1);
    cv::imshow(OPENCV_WINDOW, bgimage);
    cv::waitKey(3);
    
		cv_ptr->image = next_;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic1(0);
  ros::spin();
  return 0;
}
