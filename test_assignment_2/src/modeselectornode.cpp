#include "ros/ros.h"
#include <stdio.h>
#include "test_assignment_2/motion_node.h"

	using namespace std;
int modeselected;

int main(int argc, char **argv){
	ros::init(argc, argv, "h2_keyboard_node");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<test_assignment_2::motion_node>("motion_node_service");
	int flag = 0;
	test_assignment_2::motion_node srv;

	while(flag != 1) {
		ROS_INFO("Please enter the angle in degrees: \n");
		scanf("%lf", &srv.request.angle);

		ROS_INFO("Please enter the direction: 1. Up 2. Down\n");
		scanf("%ld", &srv.request.dir);

		if(client.call(srv)){
/*			switch(srv.response.success){
				case 1:*/
					cout << "Moved " << srv.response.angle << " degrees\n";
/*					break;
				case 3.8452:
					ROS_INFO("Exiting the program");
					flag = 1;
					break;
				default:
					ROS_INFO("Invalid input\n");
			}*/
		}
		else{
			ROS_ERROR("Failed to call service motion_node");
			return 1;
		}
	}

	return 0;
}

