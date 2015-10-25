#include "ros/ros.h"
#include <stdio.h>
#include "kuchbhi_assignment_4/motion_node.h"

int modeselected;

int main(int argc, char **argv){
	ros::init(argc, argv, "motion_mode_keyboard");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<kuchbhi_assignment_4::motion_node>("motion_node_service");

	kuchbhi_assignment_4::motion_node srv;
	ROS_INFO("Please enter the mode to be selected:\n");
	ROS_INFO("1: Farneback\n 2:MOG2\n 3:Raw Video\n");
	scanf("%ld", &srv.request.mode);
	

	if(client.call(srv)){
		switch(srv.response.mode){
			case 1:
				ROS_INFO("The mode selected is Raw Video\n");
				break;
			case 2: 
				ROS_INFO("The mode selected is Farneback\n");
				break;
			case 3:
				ROS_INFO("The mode selected is MOG2\n");
				break;
			default:
				ROS_INFO("Invalid input\n");
		}
	}
	else{
		ROS_ERROR("Failed to call service motion_node");
		return 1;
	}

	return 0;
}

