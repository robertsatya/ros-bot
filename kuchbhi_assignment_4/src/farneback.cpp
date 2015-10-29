#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;


//using namespace cv;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
	int color_mode_;
	cv::Mat next_;
	int init_;

public:
  ImageConverter(int color_mode_)
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
	this->color_mode_ = color_mode_;
    cv::namedWindow(OPENCV_WINDOW);
	next_ = Mat();
	init_ = 0;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

	void drawOptFlowMap (const Mat& flow, Mat& cflow, int step, const Scalar& color)
	{

		cflow = Mat::zeros(flow.size(),CV_8UC1);
		for(int y = 0; y < cflow.rows; y += step)
		{
			for(int x = 0; x < cflow.cols; x += step)
			{
				const Point2f& fxy = flow.at< Point2f>(y, x);
				int x0 = cvRound(x+fxy.x);
				int y0 = cvRound(y+fxy.y);
				const Point& p0 = Point((x0>=0)?x0:x,(y0>=0)?y0:y);
				if(pow(p0.x-x,2) + pow(p0.y-y,2)>pow(2*step,2))
				{
					circle(cflow, Point(x0, y0), 1, color, -1);
				}
			}
		}
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



		if(!init_)
		{
			init_ = 1;
			next_ = cv_ptr->image;
			if(color_mode_)
				cv::cvtColor(next_, next_, CV_BGR2GRAY);

			return;
		}

	    // Update GUI Window
		Mat prev_(next_);
		next_ = cv_ptr->image;

		if(color_mode_)
			cv::cvtColor(next_, next_, CV_BGR2GRAY);


    // Draw an example circle on the video stream


	    cv::imshow(OPENCV_WINDOW, next_);
			waitKey(3);
		cv_ptr->image = next_;

	  Mat flow;
  	calcOpticalFlowFarneback(prev_, next_, flow, 0.5 /*pyr_size*/ , 3 /*levels*/, 51/*win_size*/, 3 /*iterations at each pyramid*/, 5/*poly_n*/, 1.1/*poly_sigma*/, 0/*flags*/);

//		cvtColor(prev_, cflow, CV_GRAY2BGR);
  	drawOptFlowMap(flow, cflow, 5, CV_RGB(255, 255, 255));


  	createTrackbar( "D Kernel size:", "OpticalFlowFarneback", &dilation_size, 51, &Dilation );
		Dilation(dilation_size,0);


  	createTrackbar( "E Kernel size:", "OpticalFlowFarneback", &erosion_size, 51, &Erosion );
		Erosion(erosion_size,0);
	  imshow("OpticalFlowFarneback", cflow);

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic1(1);

  ros::spin();
  return 0;
}
