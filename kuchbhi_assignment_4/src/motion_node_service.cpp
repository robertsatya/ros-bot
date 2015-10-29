#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include "kuchbhi_assignment_4/motion_node.h"

using namespace std;
using namespace cv;

static const std::string MOG2_WINDOW = "Background Subtraction Output";
cv::BackgroundSubtractorMOG2 bg(10, 13, true);
SimpleBlobDetector::Params params;
vector<KeyPoint> keypoints;
static const std::string SOURCE_WINDOW = "Source Window";
static const std::string BOUNDS_WINDOW = "Boundary Window";
Mat src, src_gray, prev_, next_, bgimage;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
static int kernel_size;
int operation_mode = 1;
int prev_mode = 1;
int erosion_elem=0, erosion_size=33, dilation_elem=0, dilation_size=33;
Mat cflow=Mat();

bool selectmode(kuchbhi_assignment_4::motion_node::Request  &req, kuchbhi_assignment_4::motion_node::Response &res);
void thresh_callback(int, void* );
void Erosion(int, void*);
void Dilation(int, void*);

class MOG2Subtracter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int color_mode_;
  int init_;

public:
  MOG2Subtracter(int color_mode_)
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &MOG2Subtracter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    //	ROS_INFO("I am here");
    this->color_mode_ = color_mode_;
    cv::namedWindow(MOG2_WINDOW);
    kernel_size = 15;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    //ros::spinOnce();
    //pMOG2(5, 10, true); //MOG2 approach,
  }

  ~MOG2Subtracter()
  {
    cv::destroyWindow(MOG2_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    //ROS_INFO("I am here too");
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

    if(operation_mode == 2) {
    	if(prev_mode != 2) {
    		init_ = 0;
    		prev_mode = operation_mode;
    	}
    	if(!init_)
			{
				init_ = 1;
				next_ = cv_ptr->image;
				cv::cvtColor(next_, next_, CV_BGR2GRAY);

				return;
			}
			prev_ = next_;
			next_ = cv_ptr->image;
			cv::cvtColor(next_, next_, CV_BGR2GRAY);

			//cv_ptr->image = next_;
			//imshow("new", next_);
			waitKey(3);
	  	Mat flow;
	  	cout << prev_.size() << next_.size() << prev_.channels() << next_.channels() << endl;
  		calcOpticalFlowFarneback(prev_, next_, flow, 0.5 /*pyr_size*/ , 3 /*levels*/, 51/*win_size*/, 3 /*iterations at each pyramid*/, 5/*poly_n*/, 1.1/*poly_sigma*/, 0/*flags*/);

	  	drawOptFlowMap(flow, cflow, 5, CV_RGB(255, 255, 255));


  		createTrackbar( "D Kernel size:", "OpticalFlowFarneback", &dilation_size, 51, &Dilation );
			Dilation(dilation_size,0);


  		createTrackbar( "E Kernel size:", "OpticalFlowFarneback", &erosion_size, 51, &Erosion );
			Erosion(erosion_size,0);
			src_gray = cflow;
			cout << src_gray.size() << endl;
    }

    if(operation_mode == 1) {
    	next_ = cv_ptr->image;
    	imshow(MOG2_WINDOW, next_);
    	waitKey(3);
    	//ROS_INFO("I am here");
    	return;
    }

    if (operation_mode == 3) {
	    next_ = cv_ptr->image;
	    src = next_;
	    cv::cvtColor(next_, next_, CV_RGB2GRAY);

	    bg(next_, bgimage, -1);
	    //cv::imshow(MOG2_WINDOW, bgimage);
	    cv::waitKey(30);

	    medianBlur(bgimage, bgimage, 5);
	    //cv::imshow("MEdianBlur", bgimage);

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
	    //imshow("ED Image", bgimage);

	    src_gray = bgimage;
	  }
    GaussianBlur( src_gray, src_gray, Size(kernel_size,kernel_size), 0, 0 );
	  src = cv_ptr->image;
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
    //imshow("Random crap", contourImage);
    src_gray = contourImage;

    //return;
    keypoints.clear();

    createTrackbar( " Threshold:", MOG2_WINDOW, &thresh, max_thresh, thresh_callback );
    thresh_callback(0, 0);

    cv_ptr->image = bgimage;
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
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
				if(pow(p0.x-x,2) + pow(p0.y-y,2)>pow(step,2))
				{
					circle(cflow, Point(x0, y0), 1, color, -1);
				}
			}
		}
	}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "motion_node");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("motion_node_service",selectmode);
	MOG2Subtracter modder(0);
	ros::spin();

	return 0;
}

bool selectmode(kuchbhi_assignment_4::motion_node::Request  &req, kuchbhi_assignment_4::motion_node::Response &res)
{
	res.mode = req.mode;
	ROS_INFO("\n Selected mode is %ld", res.mode);
	prev_mode = operation_mode;
	operation_mode = res.mode;
	if (res.mode > 3) {
		exit(0);
	}
	return true;
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

  //namedWindow( BOUNDS_WINDOW, WINDOW_AUTOSIZE );
  imshow( MOG2_WINDOW, drawing );
}

/**  @function Erosion  */
void Erosion(int, void*)
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( cflow, cflow, element );
}

/** @function Dilation */
void Dilation(int, void*)
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( cflow, cflow, element );
}

