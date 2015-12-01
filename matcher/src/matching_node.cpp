#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include "objectDetector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/video.hpp>

using namespace std;

int main(int argc, char const *argv[])
{

	ros::init(argc, argv, "matching_node");
	ros::NodeHandle n;
	image_transport::Subscriber image_sub;
	//image_sub = n.subscribe("/camera/image_raw", 10, )

	ros::spin();

	return 0;
}