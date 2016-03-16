#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <csignal>
#include <cmath>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <control_node/BroadSearch.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;


void sigHandle(int sig_id);

class DisparityTrack
{
	ros::NodeHandle n;
	ros::Subscriber image_sub1, image_sub2;
	ros::Publisher left_point_pub;
	int lower_thresh[3], upper_thresh[3];
	int glower_thresh[3], gupper_thresh[3];
	double prev_left_pos[2], prev_right_pos[2];
	double gprev_left_pos[2], gprev_right_pos[2];
	Mat Pj_left, Pj_right, left_pt, right_pt, gleft_pt, gright_pt;
	double Pjl[3][4], Pjr[3][4];
	double x_pos, y_pos, depth;
	double gx_pos, gy_pos, gdepth;
	bool isConsistent;

public:
	DisparityTrack() {
		left_point_pub = n.advertise<geometry_msgs::PointStamped>("left_point", 5);
		image_sub1 = n.subscribe("/left_cam/image_raw", 100, &DisparityTrack::left_cb, this);
		image_sub2 = n.subscribe("/right_cam/image_raw", 100, &DisparityTrack::right_cb, this);
		lower_thresh[0] = 4; lower_thresh[1] = 113; lower_thresh[2] = 51;
		upper_thresh[0] = 18; upper_thresh[1] = 203; upper_thresh[2] = 250;
		glower_thresh[0] = 37; glower_thresh[1] = 108; glower_thresh[2] = 44;
		gupper_thresh[0] = 54; gupper_thresh[1] = 215; gupper_thresh[2] = 255;
		// glower_thresh[0] = 4; glower_thresh[1] = 75; glower_thresh[2] = 60;
		// gupper_thresh[0] = 18; gupper_thresh[1] = 184; gupper_thresh[2] = 196;
		// lower_thresh[0] = 37; lower_thresh[1] = 160; lower_thresh[2] = 44;
		// upper_thresh[0] = 52; upper_thresh[1] = 215; upper_thresh[2] = 255;
		x_pos = 0; y_pos = 0; depth = 0;

		Pjl[0][0] = 864.071360; Pjl[0][1] = 0; Pjl[0][2] = 271.838741; Pjl[0][3] = 0;
		Pjl[1][0] = 0; Pjl[1][1] = 864.071360; Pjl[1][2] = 235.457853; Pjl[1][3] = 0;
		Pjl[2][0] =	0; Pjl[2][1] = 0; Pjl[2][2] = 1; Pjl[2][3] = 0;

		Pj_left = Mat(3, 4, DataType<double>::type, &Pjl);

		Pjr[0][0] = 864.071360; Pjr[0][1] = 0; Pjr[0][2] = 271.838741; Pjr[0][3] = 15693.883800;
		Pjr[1][0] = 0; Pjr[1][1] = 864.071360; Pjr[1][2] = 235.457853; Pjr[1][3] = 0;
		Pjr[2][0] =	0; Pjr[2][1] = 0; Pjr[2][2] = 1; Pjr[2][3] = 0;

		Pj_right = Mat(3, 4, DataType<double>::type, &Pjr);

		namedWindow("Control Window", WINDOW_AUTOSIZE);
		createTrackbar("H_min", "Control Window", &lower_thresh[0], 255);
		createTrackbar("S_min", "Control Window", &lower_thresh[1], 255);
		createTrackbar("V_min", "Control Window", &lower_thresh[2], 255);
		createTrackbar("H_max", "Control Window", &upper_thresh[0], 255);
		createTrackbar("S_max", "Control Window", &upper_thresh[1], 255);
		createTrackbar("V_max", "Control Window", &upper_thresh[2], 255);

		prev_left_pos[0] = 0; prev_left_pos[1] = 0;
		prev_right_pos[0] = 0; prev_right_pos[1] = 0;
		left_pt = cv::Mat(1,1,CV_64FC2);
		right_pt = cv::Mat(1,1,CV_64FC2);
		
		gprev_left_pos[0] = 0; gprev_left_pos[1] = 0;
		gprev_right_pos[0] = 0; gprev_right_pos[1] = 0;
		gleft_pt = cv::Mat(1,1,CV_64FC2);
		gright_pt = cv::Mat(1,1,CV_64FC2);

	}

	void left_cb(const sensor_msgs::Image& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		Mat hsv, masked, gmasked, thr_img, fin;
		vector<Vec3f> balls;
		Mat img = cv_ptr->image;
		cvtColor(img, hsv, CV_BGR2HSV);

		inRange(hsv, Scalar(lower_thresh[0], lower_thresh[1], lower_thresh[2]),
			Scalar(upper_thresh[0], upper_thresh[1], upper_thresh[2]), masked);

		inRange(hsv, Scalar(glower_thresh[0], glower_thresh[1], glower_thresh[2]),
			Scalar(gupper_thresh[0], gupper_thresh[1], gupper_thresh[2]), gmasked);

		GaussianBlur( masked, masked, Size(9, 9), 2, 2 );
		GaussianBlur( gmasked, gmasked, Size(9, 9), 2, 2 );


		int erosion_size = 9;
		erode(masked, masked, getStructuringElement(MORPH_ERODE,
													Size( 2*erosion_size + 1, 2*erosion_size+1 ),
													Point(erosion_size, erosion_size)) );
		dilate(masked, masked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		erosion_size = 5;
		erode(gmasked, gmasked, getStructuringElement(MORPH_ERODE,
													Size( 2*erosion_size + 1, 2*erosion_size+1 ),
													Point(erosion_size, erosion_size)) );
		dilate(gmasked, gmasked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		thr_img = masked;

		// HoughCircles( masked, balls, CV_HOUGH_GRADIENT, 1, 10, 50, 30, 0, 0 );

		cv::bitwise_and(img, img, fin, thr_img);

		erode(gmasked, gmasked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		dilate(gmasked, gmasked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		Moments moments = cv::moments(masked, false);
		Moments gmoments = cv::moments(gmasked, false);

		// SimpleBlobDetector detector;
		// std::vector<KeyPoint> points;
		// detector.detect(masked, points);

		// drawKeypoints( fin, points, fin, Scalar(255, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


		if(moments.m00 > 0) {
			prev_left_pos[0] = moments.m10/moments.m00;
			prev_left_pos[1] = moments.m01/moments.m00;
			int cx = prev_left_pos[0];
			int cy = prev_left_pos[1];

			left_pt.at<cv::Vec2d>(0,0)[0] = cx;
			left_pt.at<cv::Vec2d>(0,0)[1] = cy;

			cv::circle(fin, cv::Point(cx, cy), 10, Scalar(0, 0, 255), 2);
			// draw(fin, balls);
			// if (balls.size() != 0)
			// 	cout << "count: " << balls.size() << endl;
		} else {
			left_pt.at<cv::Vec2d>(0,0)[0] = 0;
			prev_left_pos[0] = 0;
			left_pt.at<cv::Vec2d>(0,0)[1] = 0;
			prev_left_pos[1] = 0;
		}

		if(gmoments.m00 > 0) {
			int cx = gmoments.m10/gmoments.m00;
			int cy = gmoments.m01/gmoments.m00;
			
			gleft_pt.at<cv::Vec2d>(0,0)[0] = cx;
			gprev_left_pos[0] = cx;
			gleft_pt.at<cv::Vec2d>(0,0)[1] = cy;
			gprev_left_pos[1] = cy;

			cv::circle(fin, cv::Point(cx, cy), 10, Scalar(0, 255, 0), 2);
			// draw(fin, balls);
			// if (balls.size() != 0)
			// 	cout << "count: " << balls.size() << endl;
		} else {
			gleft_pt.at<cv::Vec2d>(0,0)[0] = 0;
			gprev_left_pos[0] = 0;
			gleft_pt.at<cv::Vec2d>(0,0)[1] = 0;
			gprev_left_pos[1] = 0;
		}

		imshow("Thresh Image", fin);

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
		Mat hsv, masked, gmasked;
		Mat img = cv_ptr->image;
		cvtColor(img, hsv, CV_BGR2HSV);

		inRange(hsv, Scalar(lower_thresh[0], lower_thresh[1], lower_thresh[2]),
			Scalar(upper_thresh[0], upper_thresh[1], upper_thresh[2]), masked);

		GaussianBlur( hsv, hsv, Size(9, 9), 2, 2 );

		erode(masked, masked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		dilate(masked, masked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		Moments moments = cv::moments(masked, false);

		inRange(hsv, Scalar(lower_thresh[0], lower_thresh[1], lower_thresh[2]),
			Scalar(upper_thresh[0], upper_thresh[1], upper_thresh[2]), gmasked);

		erode(gmasked, gmasked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		dilate(gmasked, gmasked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		Moments gmoments = cv::moments(gmasked, false);


		if(moments.m00 > 0 && (prev_left_pos[0] > 0 && prev_left_pos[1] > 0)) {
			double cx = moments.m10/moments.m00;
			double cy = moments.m01/moments.m00;

			prev_right_pos[0] = cx;
			prev_right_pos[1] = cy;

			right_pt.at<cv::Vec2d>(0,0)[0] = cx;
			right_pt.at<cv::Vec2d>(0,0)[1] = cy;

			Mat out = Mat(4, 1, DataType<double>::type);
			cv::triangulatePoints(Pj_left, Pj_right, left_pt, right_pt, out);

			out = out / out.at<double>(0,3);

			x_pos = out.at<double>(0,0) + 9.0;
			y_pos = out.at<double>(0,1);
			depth = out.at<double>(0,2);

			postLeftPoint(x_pos, y_pos, depth);
		}

		waitKey(3);
	}

	void postLeftPoint (double x, double y, double depth) {
		geometry_msgs::PointStamped point;
		point.header.frame_id = "/left_camera";
		point.header.stamp = ros::Time().now();
		point.point.x = x_pos;
		point.point.y = y_pos;
		point.point.z = depth;

		left_point_pub.publish(point);
	}

	bool send_loc(control_node::BroadSearch::Request &req, control_node::BroadSearch::Response &res) {
		isConsistent = false;
		int count = 0;
		//while(!isConsistent) {
		//	int prev = depth;

		//}
		cout << "Info about ball sent" << endl;
		printf("%f %f %f\n", x_pos, y_pos, depth);
		res.x = x_pos;
		res.y = y_pos;
		res.depth = depth;
		return true;
	}

	 void draw(cv::Mat& mat, const std::vector<cv::Vec3f>& container)
	{
		if(!container.empty())
			for(unsigned i = 0; i != container.size() && i != 4; ++i)
				if(container[i][2] > 5 && container[i][2] < 80)
					cv::circle(mat, cv::Point(container[i][0], container[i][1]), 45, cv::Scalar(255,0,0), 3);
	}

};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "depth_tracking");
	signal(SIGINT, sigHandle);
	signal(SIGTERM, sigHandle);
	DisparityTrack dt = DisparityTrack();
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("broad_search_service", &DisparityTrack::send_loc, &dt);
	ros::spin();
	return 0;
}

void sigHandle(int sig_id) {
	cout << "KIller !! :(" << endl;
	exit(sig_id);
}
