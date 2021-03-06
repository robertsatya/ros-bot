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
// #include <control_node/BroadSearch.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;

static double x_pos, y_pos, depth;

void sigHandle(int sig_id);

class DisparTrack
{
	ros::NodeHandle n;
	ros::Subscriber image_sub1, image_sub2;
	ros::Publisher left_point_pub;
	int lower_thresh[3], upper_thresh[3];
	int glower_thresh[3], gupper_thresh[3];
	double prev_left_pos[2], prev_right_pos[2];
	Mat Pj_left, Pj_right, left_pt, right_pt;
	double Pjl[3][4], Pjr[3][4];
	double x_pos, y_pos, depth;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

public:
	DisparTrack() {
		left_point_pub = n.advertise<geometry_msgs::PointStamped>("buck_point", 5);
		image_sub1 = n.subscribe("/left_cam/image_raw", 100, &DisparTrack::left_cb, this);
		image_sub2 = n.subscribe("/right_cam/image_raw", 100, &DisparTrack::right_cb, this);
		lower_thresh[0] = 110; lower_thresh[1] = 87; lower_thresh[2] = 47;
		upper_thresh[0] = 121; upper_thresh[1] = 175; upper_thresh[2] = 95;
		x_pos = 0; y_pos = 0; depth = 0;

		Pjl[0][0] = 870.1896695734192; Pjl[0][1] = 0; Pjl[0][2] = 345.4360542297363; Pjl[0][3] = 0;
		Pjl[1][0] = 0; Pjl[1][1] = 870.1896695734192; Pjl[1][2] = 260.3978462219238; Pjl[1][3] = 0;
		Pjl[2][0] =	0; Pjl[2][1] = 0; Pjl[2][2] = 1; Pjl[2][3] = 0;

		Pj_left = Mat(3, 4, DataType<double>::type, &Pjl);

		Pjr[0][0] = 870.1896695734192; Pjr[0][1] = 0; Pjr[0][2] = 345.4360542297363; Pjr[0][3] = 13321.68894098538;
		Pjr[1][0] = 0; Pjr[1][1] = 870.1896695734192; Pjr[1][2] = 260.3978462219238; Pjr[1][3] = 0;
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

		// inRange(hsv, Scalar(glower_thresh[0], glower_thresh[1], glower_thresh[2]),
		// 	Scalar(gupper_thresh[0], gupper_thresh[1], gupper_thresh[2]), gmasked);

		GaussianBlur( masked, masked, Size(9, 9), 2, 2 );
		// GaussianBlur( gmasked, gmasked, Size(9, 9), 2, 2 );
		int erosion_size = 7;
		erode(masked, masked, getStructuringElement(MORPH_ERODE,
													Size( 2*erosion_size + 1, 2*erosion_size+1 ),
													Point(erosion_size, erosion_size)) );
		dilate(masked, masked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		thr_img = masked;

		//findContours(thr_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// HoughCircles( masked, balls, CV_HOUGH_GRADIENT, 1, 10, 50, 30, 0, 0 );

		cv::bitwise_and(img, img, fin, thr_img);

		int idx = 0;
		//for( ; idx >= 0; idx = hierarchy[idx][0] )
		//{
		//	Scalar color( rand()&255, rand()&255, rand()&255 );
		//	drawContours( fin, contours, -1, color );
		//}

		// erode(gmasked, gmasked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		// dilate(gmasked, gmasked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		Moments moments = cv::moments(masked, false);
		// Moments gmoments = cv::moments(gmasked, false);

		//SimpleBlobDetector detector;
		//std::vector<KeyPoint> points;
		//detector.detect(masked, points);

		//drawKeypoints( fin, points, fin, Scalar(255, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );


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
		Mat hsv, masked;
		Mat img = cv_ptr->image;
		cvtColor(img, hsv, CV_BGR2HSV);

		inRange(hsv, Scalar(lower_thresh[0], lower_thresh[1], lower_thresh[2]),
			Scalar(upper_thresh[0], upper_thresh[1], upper_thresh[2]), masked);

		GaussianBlur( hsv, hsv, Size(9, 9), 2, 2 );

		erode(masked, masked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		dilate(masked, masked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		Moments moments = cv::moments(masked, false);

		if(moments.m00 > 0) {
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
		}

		postLeftPoint(0, 0, 0);

		waitKey(3);
	}


	void postLeftPoint (double x, double y, double depth) {

		geometry_msgs::PointStamped point;
		point.header.frame_id = "/left_camera";
		point.header.stamp = ros::Time().now();

		int color = 0;

		cout << "Info about ball" << endl;
		printf("%f %f %f\n", x_pos, y_pos, depth);

		if (depth > 0) {
			// cout << "Info about ball sent" << endl;
			// printf("%f %f %f\n", x_pos, y_pos, depth);
			point.point.x = x_pos;
			point.point.y = y_pos;
			point.point.z = depth;
		} else {
			point.point.x = 0;
			point.point.y = 0;
			point.point.z = -1;
		}

		// point.point.x = x_pos;
		// point.point.y = y_pos;
		// point.point.z = depth;

		left_point_pub.publish(point);
	}

	// bool send_loc(control_node::BroadSearch::Request &req, control_node::BroadSearch::Response &res) {
	// 	res.x = x_pos;
	// 	res.y = y_pos;
	// 	res.depth = depth;
	// 	return true;
	// }

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
	ros::init(argc, argv, "goal_search");
	signal(SIGINT, sigHandle);
	signal(SIGTERM, sigHandle);
	DisparTrack dt = DisparTrack();
	ros::NodeHandle nh;
	// ros::ServiceServer service = nh.advertiseService("goal_search_service", &DisparTrack::send_loc, &dt);
	ros::spin();
	return 0;
}

void sigHandle(int sig_id) {
	cout << "KIller !! :(" << endl;
	exit(sig_id);
}
