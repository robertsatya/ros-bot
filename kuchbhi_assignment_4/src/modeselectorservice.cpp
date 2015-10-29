#include "ros/ros.h"
#include <stdio.h>
#include "kuchbhi_assignment_4/modeselector.h"


void thresh_callback(int, void* );

bool selectmode(kuchbhi_assignment_4::modeselector::Request  &req, kuchbhi_assignment_4::modeselector::Response &res);

int main(int argc, char **argv){
	ros::init(argc, argv, "modeselector_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("modeselector",selectmode);
	ROS_INFO("Ready to select mode\n");
	ros::spin();

	return 0;
}

bool selectmode(kuchbhi_assignment_4::modeselector::Request  &req, kuchbhi_assignment_4::modeselector::Response &res){
	ROS_INFO("Please enter the mode to be selected:\n");
	ROS_INFO("1: Farneback\n 2:MOG2\n 3:Raw Video\n");
	scanf("%ld", &res.mode);
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