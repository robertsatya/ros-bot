#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <rob_nav/MyNode.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rob_nav/navigationAction.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nav_client");

	MyNode my_node;
	geometry_msgs::PointStamped p;
	int i=1,mode=0;
	while(ros::ok())
	{
  	p.header.seq = i;
  	p.header.stamp = ros::Time::now();
  	p.header.frame_id = "/robot";
		cout << "Please enter forward coordinate";
  	cin >> p.point.x;
		cout << "Please enter mode: 1) Forward 2) Circular";
  	cin >> mode;

  	p.point.y = 0;
  	p.point.z = 0;
    my_node.doStuff(p,mode);
  	while(my_node.fin<3)
  	{
  	}
		i++;
	}
 // ros::spinOnce();
  //exit
  return 0;
}
