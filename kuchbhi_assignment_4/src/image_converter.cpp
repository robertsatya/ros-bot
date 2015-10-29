#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
//#include "opencv2/imgcodecs/imgcodecs.hpp"

using namespace cv;
using namespace std;


static const std::string MOG2_WINDOW = "Background Subtraction Output";
cv::BackgroundSubtractorMOG2 bg(10, 13, true);
SimpleBlobDetector::Params params;
vector<KeyPoint> keypoints;

void thresh_callback(int, void* );
void gaussian_callback(int, void* );

static const std::string SOURCE_WINDOW = "Source Window";
static const std::string BOUNDS_WINDOW = "Boundary Window";
Mat src, src_gray, prev_, next_, bgimage;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
static int kernel_size;

class MOG2Subtracter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int color_mode_;

public:
  MOG2Subtracter(int color_mode_)
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &MOG2Subtracter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    this->color_mode_ = color_mode_;
    cv::namedWindow(MOG2_WINDOW);
    kernel_size = 15;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    //pMOG2(5, 10, true); //MOG2 approach,
  }

  ~MOG2Subtracter()
  {
    cv::destroyWindow(MOG2_WINDOW);
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
    src = next_;
    if(color_mode_)
      cv::cvtColor(next_, next_, CV_RGB2GRAY);

    bg(next_, bgimage, -1);
    cv::imshow(MOG2_WINDOW, bgimage);
    cv::waitKey(30);

    medianBlur(bgimage, bgimage, 5);
    cv::imshow("MEdianBlur", bgimage);

    int erosion_type = MORPH_RECT;
    int erosion_size = 2;
    Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( -1, -1 ) );
    erode( bgimage, bgimage, element );
    erosion_size = 7;
    element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( -1, -1 ) );
    dilate( bgimage, bgimage, element );
    imshow("ED Image", bgimage);

    src_gray = bgimage;
    //`GaussianBlur( src_gray, src_gray, Size(kernel_size,kernel_size), 0, 0 );

    //Aditional Processing
    //eroding the image
    Mat background;
    vector<vector<Point> > contours;
    cv::Mat element5(5,5,CV_8U,cv::Scalar(1));
    cv::morphologyEx(src_gray,src_gray,cv::MORPH_CLOSE,element5);
    src_gray.copyTo(background);
    cv::findContours(src_gray, contours, CV_RETR_EXTERNAL,
    CV_CHAIN_APPROX_NONE );
    //Draw the contours
    cv::Mat contourImage = cv::Mat::zeros(src_gray.rows,src_gray.cols,CV_8UC1);
    cv::drawContours(contourImage, contours, -1, cv::Scalar(255));
    imshow("Random crap", contourImage);
    src_gray = contourImage;

    //return;
    keypoints.clear();

    createTrackbar( " Threshold:", BOUNDS_WINDOW, &thresh, max_thresh, thresh_callback );
    thresh_callback(0, 0);

    cv_ptr->image = bgimage;
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  MOG2Subtracter ic1(0);
  // BoundaryMaker ic2(0);
  ros::spin();
  return 0;
}


void gaussian_callback(int, void* )
{
  if (kernel_size%2 == 0)
    kernel_size+=1;
  GaussianBlur( src_gray, src_gray, Size(kernel_size,kernel_size), 0, 0 );
}

void thresh_callback(int, void* )
{
  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
  findContours( threshold_output, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_TC89_KCOS, Point(0, 0) );

  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( size_t i = 0; i < contours.size(); i++ )
  {
    approxPolyDP( Mat(contours[i]), contours_poly[i], 7, false );
    minEnclosingCircle( contours_poly[i], center[i], radius[i] );
  }

  // Point diff;
  // for (int i = 0; i < center.size(); ++i)
  // {
  //   for (int j = 0; j < center.size(); ++j)
  //   {
  //     diff = center[i] - center[j];
  //     if(i != j && i > j && cv::sqrt(diff.x*diff.x + diff.y*diff.y) < 50)
  //     {
  //       contours_poly[i].insert(contours_poly[i].end(), contours_poly[j].begin(), contours_poly[j].end());
  //       contours_poly[j].clear();
  //     }
  //   }
  // }

  for( size_t i = 0; i < contours.size(); i++ )
  {
    approxPolyDP( Mat(contours[i]), contours_poly[i], 7, false );
    if(contourArea(contours_poly[i]) < 3600)
      continue;
    boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    minEnclosingCircle( contours_poly[i], center[i], radius[i] );
  }

  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  src.copyTo(drawing);
  for( size_t i = 0; i< contours_poly.size(); i++ )
  {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    //drawContours( drawing, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
    rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
    //circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
  }

  namedWindow( BOUNDS_WINDOW, WINDOW_AUTOSIZE );
  imshow( BOUNDS_WINDOW, drawing );
}




