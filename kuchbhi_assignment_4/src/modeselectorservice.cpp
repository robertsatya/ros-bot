#include "ros/ros.h"
#include <stdio.h>
#include "kuchbhi_assignment_4/modeselector.h"

bool selectmode(kuchbhi_assignment_4::modeselector::Request  &req, kuchbhi_assignment_4::modeselector::Response &res){
	ROS_INFO("Please enter the mode to be selected:\n");
	ROS_INFO("1: Farneback\n 2:MOG2\n 3:Raw Video\n");
	scanf("%ld", &res.mode);
	//ROS_INFO("The mode selected is %d\n", res.mode);
//	return res.mode;
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "modeselector_server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("modeselector",selectmode);
	ROS_INFO("Ready to select mode\n");
	ros::spin();

	return 0;
}


