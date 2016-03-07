#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <rob_nav/Arm.h>

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nav_arm_client");

	uint8_t act=1;
	ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<rob_nav::Arm>("nav_arm");
  rob_nav::Arm srv;
  srv.request.act = act;
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.status);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
	
 // ros::spinOnce();
  //exit
  return 0;
}
