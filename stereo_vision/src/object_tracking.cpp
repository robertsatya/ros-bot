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


float disparity_ratio;
int disparity_ratio_int;
void updateDisp(int, void* );

class ObjectTrack
{
	ros::NodeHandle n;
	ros::Subscriber image_sub;
	ros::Publisher left_point_pub, right_point_pub;
	float x_center, y_center;
	float img_width, img_height;
	int lower_thresh[3], upper_thresh[3];
	float prev_left_pos[2];
	float f;
	Mat K;

public:
	ObjectTrack() {
		left_point_pub = n.advertise<geometry_msgs::PointStamped>("left_point", 5);
		right_point_pub = n.advertise<geometry_msgs::PointStamped>("right_point", 5);
		image_sub = n.subscribe("/left_cam/image_raw", 100, &ObjectTrack::left_cb, this);
		image_sub = n.subscribe("/right_cam/image_raw", 100, &ObjectTrack::right_cb, this);
		lower_thresh[0] = 39; lower_thresh[1] = 68; lower_thresh[2] = 163;
		upper_thresh[0] = 79; upper_thresh[1] = 222; upper_thresh[2] = 255;
		f = 640.0;

		img_width = 640.0;
		img_height = 480.0;
		x_center = img_width/2; y_center = img_height/2;
		double Kmat[3][3] = {822.324161923132, 0, 335.9440815662755, 0, 837.2065020719881, 199.7435926780396, 0, 0, 1};
		K = Mat(3, 3, DataType<double>::type, &Kmat);
		cv::invert(K, K);
		//namedWindow("Disp Control", WINDOW_AUTOSIZE);
		//createTrackbar("Disparity Ratio", "Disp Control", &disparity_ratio_int, 500, updateDisp);
		//updateDisp(disparity_ratio_int, 0);
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

			postLeftPoint(prev_left_pos[0], prev_left_pos[1]);
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
			float cx = moments.m10/moments.m00;
			float cy = moments.m01/moments.m00;

			postRightPoint(cx, cy);
		}

		waitKey(3);
	}

	void postLeftPoint (float x, float y) {
		float _x[3] = {x, y, 1};
		Mat pos = cv::Mat(3, 1, DataType<double>::type, *_x);
		//transpose(pos, pos);
		Mat worldPos = K * pos;

		geometry_msgs::PointStamped point;
		point.header.frame_id = "/left_camera";
		point.header.stamp = ros::Time();
		point.point.x = worldPos.at<float>(0);
		point.point.y = worldPos.at<float>(1);
		point.point.z = worldPos.at<float>(2);

		left_point_pub.publish(point);
	}

	void postRightPoint (float x, float y) {
		float _x[3] = {x, y, 1};
		Mat pos = cv::Mat(3, 1, DataType<double>::type, *_x);
		//transpose(pos, pos);
		Mat worldPos = K * pos;

		geometry_msgs::PointStamped point;
		point.header.frame_id = "/right_camera";
		point.header.stamp = ros::Time();
		point.point.x = worldPos.at<float>(0);
		point.point.y = worldPos.at<float>(1);
		point.point.z = worldPos.at<float>(2);

		right_point_pub.publish(point);
	}


};



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "object_tracking");
	ObjectTrack dt = ObjectTrack();
	ros::spin();
	return 0;
}

void updateDisp(int value, void* ) {
	disparity_ratio = (float)value*0.1;
}