#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

void thresh_callback(int, void* );

static const std::string SOURCE_WINDOW = "Source Window";
static const std::string BOUNDS_WINDOW = "Boundary Window";
Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);

class BoundaryMaker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int color_mode_;

  public:
    BoundaryMaker(int color_mode_) : it_(nh_)
    {
      image_sub_ = it_.subscribe("/camera/image_raw", 1, &BoundaryMaker::imageCb, this);
      image_pub_ = it_.advertise("/image_converter/output_video", 1);
      this->color_mode_ = color_mode_;
      cv::namedWindow(SOURCE_WINDOW, WINDOW_AUTOSIZE);
    }

    ~BoundaryMaker()
    {
      cv::destroyWindow(SOURCE_WINDOW);
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

      src = cv_ptr->image;
      cvtColor( src, src_gray, CV_BGR2GRAY );
      GaussianBlur( src_gray, src_gray, Size(51,51), 0, 0 );

      createTrackbar( " Threshold:", BOUNDS_WINDOW, &thresh, max_thresh, thresh_callback );
      thresh_callback(0, 0);

      cv::imshow(SOURCE_WINDOW, cv_ptr->image);
      cv::waitKey(3);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "boundaryMaker");
  BoundaryMaker ic1(0);
  ros::spin();
  return 0;
}

void thresh_callback(int, void* )
{
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
  findContours( threshold_output, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_TC89_KCOS, Point(0, 0) );

  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( size_t i = 0; i < contours.size(); i++ )
  {
    approxPolyDP( Mat(contours[i]), contours_poly[i], 3, false );
    boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    minEnclosingCircle( contours_poly[i], center[i], radius[i] );
  }

  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( size_t i = 0; i< contours.size(); i++ )
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    drawContours( drawing, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
  }

  namedWindow( BOUNDS_WINDOW, WINDOW_AUTOSIZE );
  imshow( BOUNDS_WINDOW, drawing );
}
