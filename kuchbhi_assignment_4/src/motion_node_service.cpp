#include "ros/ros.h"
#include "kuchbhi_assignment_4/motion_node.h"

bool selectmode(kuchbhi_assignment_4::motion_node::Request  &req, kuchbhi_assignment_4::motion_node::Response &res){
	
	res.mode = req.mode;
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "motion_node");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("motion_node_service",selectmode);
	ros::spin();

	return 0;
}


