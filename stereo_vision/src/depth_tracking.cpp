#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;


double disparity_ratio;
int disparity_ratio_int;
void updateDisp(int, void* );

class DisparityTrack
{
	ros::NodeHandle n;
	ros::Subscriber image_sub1, image_sub2;
	ros::Publisher left_point_pub;
	double x_center, y_center;
	double img_width, img_height;
	int lower_thresh[3], upper_thresh[3];
	double prev_left_pos[2];
	double dx, depth;
	double Kmat[3][3];
	Mat K;

public:
	DisparityTrack() {
		left_point_pub = n.advertise<geometry_msgs::PointStamped>("left_point", 5);
		image_sub1 = n.subscribe("/left_cam/image_raw", 100, &DisparityTrack::left_cb, this);
		image_sub2 = n.subscribe("/right_cam/image_raw", 100, &DisparityTrack::right_cb, this);
		lower_thresh[0] = 3; lower_thresh[1] = 113; lower_thresh[2] = 123;
		upper_thresh[0] = 18; upper_thresh[1] = 222; upper_thresh[2] = 255;
		disparity_ratio = 211.1;
		dx = 1;

		img_width = 640.0;
		img_height = 480.0;
		x_center = img_width/2; y_center = img_height/2;
		Kmat[0][0] = 822.324161923132; Kmat[0][1] = 0; Kmat[0][2] = 335.9440815662755;
		Kmat[1][0] = 0; Kmat[1][1] = 837.2065020719881; Kmat[1][2] = 199.7435926780396;
		Kmat[2][0] = 0; Kmat[2][1] = 0; Kmat[2][2] = 1;
		K = Mat(3, 3, DataType<double>::type, &Kmat);
		// cout << K << endl;
		K = K.inv();
		// cout << K << endl;
		namedWindow("Disp Control", WINDOW_AUTOSIZE);
		disparity_ratio_int = disparity_ratio*10;
		createTrackbar("Disparity Ratio", "Disp Control", &disparity_ratio_int, 500, updateDisp);
		//updateDisp(disparity_ratio_int, 0);
		prev_left_pos[0] = 0; prev_left_pos[1] = 0;
	}

	void left_cb(const sensor_msgs::Image& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		Mat hsv, masked;
		Mat img = cv_ptr->image;
		cvtColor(img, hsv, CV_BGR2HSV);
		inRange(hsv, Scalar(lower_thresh[0], lower_thresh[1], lower_thresh[2]),
			Scalar(upper_thresh[0], upper_thresh[1], upper_thresh[2]), masked);

		erode(masked, masked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		dilate(masked, masked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		Moments moments = cv::moments(masked, false);

		if(moments.m00 > 0) {
			prev_left_pos[0] = moments.m10/moments.m00;
			prev_left_pos[1] = moments.m01/moments.m00;
			int cx = prev_left_pos[0];
			int cy = prev_left_pos[1];
			cv::circle(img, cv::Point(cx, cy), 10, Scalar(0, 0, 255), 2);
		}

		waitKey(3);
	}

	void right_cb(const sensor_msgs::Image& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		Mat hsv, masked;
		Mat img = cv_ptr->image;
		cvtColor(img, hsv, CV_BGR2HSV);
		inRange(hsv, Scalar(lower_thresh[0], lower_thresh[1], lower_thresh[2]),
			Scalar(upper_thresh[0], upper_thresh[1], upper_thresh[2]), masked);

		erode(masked, masked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		dilate(masked, masked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		Moments moments = cv::moments(masked, false);

		if(moments.m00 > 0) {
			double cx = moments.m10/moments.m00;
			double cy = moments.m01/moments.m00;

			dx = (double)(prev_left_pos[0] - cx);
			depth = disparity_ratio/dx;

			//cout << prev_left_pos[0] << " "<< prev_left_pos[1] <<" " << depth << endl;

			postLeftPoint(prev_left_pos[0], prev_left_pos[1], depth);
		}

		waitKey(3);
	}

	void postLeftPoint (double x, double y, double depth) {
		//cout << "***********start**************" << endl;
		double _x[3] = {x, y, 1.0};
		//cout << K << endl;
		//cout << _x[0] << "  " << _x[1] << "  "<< _x[2] << endl;
		Mat pos = cv::Mat(3, 1, DataType<double>::type, *_x);
		// cout << pos << endl;
		// transpose(pos, pos);
		// cout << K << endl;
		Mat worldPos; //worldPos = (K * pos);// * depth ;
		//gemm(K, pos, 1.0, NULL, 0, worldPos, 0);

		double sum;
		double c[3] = {0};
		for(int i=0;i<3;i++){ //row of first matrix
			sum=0;
			for(int k=0;k<3;k++)
				sum = sum + K.at<double>(i,k) * _x[k];
			c[i]=sum*depth;
		}
		//cout << c[0] << "  " << c[1] << "  "<< c[2] << endl;
		//cout << worldPos << endl;
//		double* wData = (double*) worldPos.data;
//		cout << wData[0] << endl;
		geometry_msgs::PointStamped point;
		point.header.frame_id = "/left_camera";
		point.header.stamp = ros::Time().now();
		point.point.x = c[0]; //worldPos.at<double>(0);
		point.point.y = c[1]; //worldPos.at<double>(1);
		point.point.z = c[2]; //worldPos.at<double>(2);
//		cout << point.point.x << " " << point.point.y << " " << point.point.z << endl;
		left_point_pub.publish(point);
	}


};



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "depth_tracking");
	DisparityTrack dt = DisparityTrack();
	ros::spin();
	return 0;
}

void updateDisp(int , void* ) {
	disparity_ratio = (double)disparity_ratio_int*0.1;
}
