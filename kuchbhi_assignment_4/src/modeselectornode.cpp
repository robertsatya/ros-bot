#include "ros/ros.h"
#include <stdio.h>
#include "kuchbhi_assignment_4/motion_node.h"

int modeselected;

int main(int argc, char **argv){
	ros::init(argc, argv, "motion_mode_keyboard");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<kuchbhi_assignment_4::motion_node>("motion_node_service");
	int flag = 0;
	kuchbhi_assignment_4::motion_node srv;

	while(flag != 1) {
		ROS_INFO("Please enter the mode to be selected: \n");
		ROS_INFO("\n 1: Raw Video\n 2: Farneback\n 3: MOG2\n 4: Exit\n");
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
				case 4:
					ROS_INFO("Exiting the program");
					flag = 1;
					break;
				default:
					ROS_INFO("Invalid input\n");
			}
		}
		else{
			ROS_ERROR("Failed to call service motion_node");
			return 1;
		}
	}

	return 0;
}

