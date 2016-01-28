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


int lower_thresh[3], upper_thresh[3], f_int;
float f;
int tb[7]={0};
void updateDisp(int, void* );
void updateHmin(int, void* );
void updateSmin(int, void* );
void updateVmin(int, void* );
void updateHmax(int, void* );
void updateSmax(int, void* );
void updateVmax(int, void* );
void updateF(int, void* );

class ThreshMan
{
	ros::NodeHandle n;
	ros::Subscriber image_sub;
	ros::Publisher left_point_pub;
	float x_center, y_center;
	float img_width, img_height;
	float prev_left_pos[2];
	Mat K;

public:
	ThreshMan() {
		left_point_pub = n.advertise<geometry_msgs::PointStamped>("left_point", 5);
		image_sub = n.subscribe("/left_cam/image_raw", 100, &ThreshMan::image_cb, this);
		lower_thresh[0] = 39; lower_thresh[1] = 68; lower_thresh[2] = 163;
		upper_thresh[0] = 79; upper_thresh[1] = 222; upper_thresh[2] = 255;
		f = 1190.0;

		img_width = 640.0;
		img_height = 480.0;
		x_center = (img_width/2)/0.6906; y_center = (img_height/2)/0.6906;
		double Kmat[3][3] = {f, 0, x_center, 0, f, y_center, 0, 0, 1.0};
		K = Mat(3, 3, DataType<double>::type, &Kmat);
		namedWindow("Color Control", WINDOW_AUTOSIZE);
		namedWindow("Tracking", WINDOW_AUTOSIZE);
		for (int i = 0; i < 7; ++i)
			tb[i] = 0;
		createTrackbar("H_min", "Color Control", &tb[0], 255, updateHmin);
		createTrackbar("S_min", "Color Control", &tb[1], 255, updateSmin);
		createTrackbar("V_min", "Color Control", &tb[2], 255, updateVmin);
		createTrackbar("H_max", "Color Control", &tb[3], 255, updateHmax);
		createTrackbar("S_max", "Color Control", &tb[4], 255, updateSmax);
		createTrackbar("V_max", "Color Control", &tb[5], 255, updateVmax);
		createTrackbar("Offset", "Color Control", &tb[6], 7000, updateF);

	}

	void image_cb(const sensor_msgs::Image& msg) {
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

		// erode(masked, masked, getStructuringElement(MORPH_ERODE, Point(3,3)) );
		// dilate(masked, masked, getStructuringElement(MORPH_DILATE, Point(3,3)) );

		cv::bitwise_and(img, img, img, masked);

		Moments moments = cv::moments(masked, false);

		if(moments.m00 > 0) {
			float cx = moments.m10/moments.m00;
			float cy = moments.m01/moments.m00;

			cv::circle(img, cv::Point(cx, cy), 10, Scalar(0, 0, 255), 2);

			postLeftPoint(prev_left_pos[0], prev_left_pos[1]);
		}

		imshow("Tracking", img);

		waitKey(3);
	}

	void postLeftPoint (float x, float y) {
		float _x[3] = {x, y, 0.6906};
		Mat pos = cv::Mat(3, 1, DataType<float>::type, &_x);
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


};



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "thresh_manager");
	ThreshMan dt = ThreshMan();
	ros::spin();
	return 0;
}

void updateF(int, void* ) {
	f = (float)tb[6];
}

void updateHmin(int, void* ) {
	lower_thresh[0] = tb[0];
}
void updateSmin(int, void* ) {
	lower_thresh[1] = tb[1];
}
void updateVmin(int, void* ) {
	lower_thresh[2] = tb[2];
}
void updateHmax(int, void* ) {
	upper_thresh[0] = tb[3];
}
void updateSmax(int, void* ) {
	upper_thresh[1] = tb[4];
}
void updateVmax(int, void* ) {
	upper_thresh[2] = tb[5];
}