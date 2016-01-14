#include "ros/ros.h"
#include <stdio.h>
#include "test_assignment_1/motion_node.h"

int modeselected;

int main(int argc, char **argv){
	ros::init(argc, argv, "keyboard_node");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<test_assignment_1::motion_node>("motion_node_service");
	int flag = 0;
	test_assignment_1::motion_node srv;

	while(flag != 1) {
		ROS_INFO("Please enter the movement direction: \n");
		ROS_INFO("\n 1: Up 2: Down 3: Right 4: Left 0: Exit\n");
		scanf("%ld", &srv.request.mode);


		if(client.call(srv)){
			switch(srv.response.mode){
				case 1:
					ROS_INFO("Up\n");
					break;
				case 2:
					ROS_INFO("Down\n");
					break;
				case 3:
					ROS_INFO("Right\n");
					break;
				case 4:
					ROS_INFO("Left\n");
					break;
				case 0:
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

