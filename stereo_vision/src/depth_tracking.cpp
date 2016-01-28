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


void updateDisp(int, void* );
int disparity_ratio;

class DisparityTrack
{
	ros::NodeHandle n;
	ros::Subscriber image_sub;
	ros::Publisher left_point_pub;
	float x_center, y_center;
	float img_width, img_height;
	int lower_thresh[3], upper_thresh[3];
	float prev_left_pos[2];
	Mat K;

public:
	DisparityTrack() {
		left_point_pub = n.advertise<geometry_msgs::PointStamped>("left_point", 5);
		image_sub = n.subscribe("/left_cam/image_raw", 100, &DisparityTrack::left_cb, this);
		image_sub = n.subscribe("/right_cam/image_raw", 100, &DisparityTrack::right_cb, this);
		lower_thresh[0] = 39; lower_thresh[1] = 68; lower_thresh[2] = 163;
		upper_thresh[0] = 79; upper_thresh[1] = 222; upper_thresh[2] = 255;
		disparity_ratio = 211;

		img_width = 640.0;
		img_height = 480.0;
		x_center = img_width/2; y_center = img_height/2;
		double Kmat[3][3] = {822.324161923132, 0, 335.9440815662755, 0, 837.2065020719881, 199.7435926780396, 0, 0, 1};
		K = Mat(3, 3, DataType<double>::type, &Kmat);
		namedWindow("Disp Control", WINDOW_AUTOSIZE);
		createTrackbar("Disparity Ratio", "Disp Control", &disparity_ratio, 500, updateDisp);
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
	    Mat hsv;
	    Mat img = cv_ptr->image;
		cvtColor(img, hsv, CV_BGR2HSV);
	}

	void right_cb(const sensor_msgs::Image& msg) {

	}

};



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "depth_tracking");
	DisparityTrack dt = DisparityTrack();
	return 0;
}

void updateDisp(int, void*) {

}