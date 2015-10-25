#include "ros/ros.h"
#include "kuchbhi_assignment_4/modeselector.h"

int modeselected;

int main(int argc, char **argv){
	ros::init(argc, argv, "motion_mode_keyboard");
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<kuchbhi_assignment_4::modeselector>("modeselector");

	kuchbhi_assignment_4::modeselector srv;
	srv.request.a = (long) 5;
	
	if(client.call(srv)){
		ROS_INFO("The mode selected is %ld\n", srv.response.mode);
		modeselected = srv.response.mode;
	}
	else{
		ROS_ERROR("Failed to call service modeselector");
		return 1;
	}

	return 0;
}

