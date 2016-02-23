#include<ros/ros.h>
#include<turtlesim/Spawn.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/TeleportRelative.h>
#include<std_srvs/Empty.h>
#include<turtlesim/SetPen.h>
#include<turtlesim/Kill.h>

void teleport_relative(ros::NodeHandle &n, const char *srv_name, float l, float a)
{
	ros::service::waitForService(srv_name);
	turtlesim::TeleportRelative tr;
	tr.request.linear = l;
	tr.request.angular = a;
	if(n.serviceClient<turtlesim::TeleportRelative>(srv_name).call(tr))
	{
		ROS_INFO("'%s' moved by linear: %f angular: %f'",srv_name,tr.request.linear,tr.request.angular);
	}
	else
	{
		ROS_ERROR("teleport_relative failed for '%s'",srv_name);
	}
}

void clear(ros::NodeHandle &n, const char *srv_name)
{
	ros::service::waitForService(srv_name);
	std_srvs::Empty clear;
	if(n.serviceClient<std_srvs::Empty>(srv_name).call(clear))
	{
		ROS_INFO("Path successfully cleared");
	}
	else
	{
		ROS_ERROR("Clear failed");
	}
}

void move_turtle(ros::NodeHandle &n, const char *topic_name, ros::Rate &rate, float lx, float ly, float lz, float ax, float ay, float az)
{
	ros::Publisher velPub = n.advertise<geometry_msgs::Twist>(topic_name, 10);
	geometry_msgs::Twist vel_msg;
	geometry_msgs::Vector3 vec;
	vec.x = lx;
	vec.y = ly;
	vec.z = lz;	
	vel_msg.linear = vec;
	vec.x = ax;
	vec.y = ay;
	vec.z = az;
	vel_msg.angular = vec;
	rate.sleep();	
	velPub.publish(vel_msg);
	rate.sleep();
}

void spawn_turtle(ros::NodeHandle &n, const char *srv_name, float x, float y, float theta, const char *name)
{
	ros::service::waitForService(srv_name);
	turtlesim::Spawn spawn;
	spawn.request.x = x;
	spawn.request.y = y;
	spawn.request.theta = theta;
	spawn.request.name = name;
	if(n.serviceClient<turtlesim::Spawn>(srv_name).call(spawn))
	{
		ROS_INFO("'%s' spawned at x:%f y:%f theta:%f",spawn.request.name.c_str(),spawn.request.x,spawn.request.y,spawn.request.theta);
	}
	else
	{
		ROS_ERROR("The new turtle failed to spawn");
	}
}

void turtle_set_pen(ros::NodeHandle &n, const char *srv_name, float r, float g, float b, int width, bool off)
{
	ros::service::waitForService(srv_name);
	turtlesim::SetPen pen;
	pen.request.r = r;
	pen.request.g = g;
	pen.request.b = b;
	pen.request.width = width;
	pen.request.off = off;

	if(n.serviceClient<turtlesim::SetPen>(srv_name).call(pen))
	{
		ROS_INFO("Pen set for %s", srv_name);
	}
	else
	{
		ROS_ERROR("Pen set failed for %s", srv_name);
	}
}

void kill_turtle(ros::NodeHandle &n, const char *srv_name, const char* turtle_name)
{
	ros::service::waitForService(srv_name);
	turtlesim::Kill kill;
	kill.request.name = turtle_name;	
	if(n.serviceClient<turtlesim::Kill>(srv_name).call(kill))
	{
		ROS_INFO("%s killed successfully", turtle_name);
	}
	else
	{
		ROS_ERROR("%s could not be killed", turtle_name);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "initials");
	ros::NodeHandle n;
	ros::Rate rate(1);

	//Kill turtle1
	kill_turtle(n, "/kill", "turtle1");
	rate.sleep();

	//Spawn for N and dot
	spawn_turtle(n, "/spawn", 2, 6.5, 1.57, "turtle2");
	rate.sleep();
	spawn_turtle(n, "/spawn", 6, 6.5, 1.57, "turtle3");
	turtle_set_pen(n, "/turtle2/set_pen", 50, 250, 50, 5, false);
	turtle_set_pen(n, "/turtle3/set_pen", 50, 250, 50, 5, false);
	rate.sleep();

	//N first line
	teleport_relative(n, "/turtle2/teleport_relative", 3, 0);
	rate.sleep();

	//First dot half
	move_turtle(n, "/turtle3/cmd_vel", rate, 0.157, 0, 0, 0, 0, 3.14);

	//N second line
	teleport_relative(n, "/turtle2/teleport_relative", 4.3, 3.92);
	rate.sleep();

	//First dot complete
	move_turtle(n, "/turtle3/cmd_vel", rate, 0.157, 0, 0, 0, 0, 3.14);
	kill_turtle(n, "/kill", "turtle3");
	rate.sleep();

	//N third line
	teleport_relative(n, "/turtle2/teleport_relative", 3, 2.35);
	rate.sleep();
	kill_turtle(n, "/kill", "turtle2");
	rate.sleep();

	//Spawn for 2nd dot
	spawn_turtle(n, "/spawn", 9.5, 6.5, 1.57, "turtle3");
	turtle_set_pen(n, "/turtle3/set_pen", 50, 250, 50, 5, false);
	move_turtle(n, "/turtle3/cmd_vel", rate, 0.157, 0, 0, 0, 0, 3.14);

	//Spawn for S
	rate.sleep();
	spawn_turtle(n, "/spawn", 8.5, 4.25, 1.57, "turtle2");
	turtle_set_pen(n, "/turtle2/set_pen", 50, 250, 50, 5, false);
	move_turtle(n, "/turtle2/cmd_vel", rate, 3.5325, 0, 0, 0, 0, 4.71);

	//Finish 2nd dot
	move_turtle(n, "/turtle3/cmd_vel", rate, 0.157, 0, 0, 0, 0, 3.14);
	kill_turtle(n, "/kill", "turtle3");

	//Finish S
	move_turtle(n, "/turtle2/cmd_vel", rate, 3.5325, 0, 0, 0, 0, -4.71);
	kill_turtle(n, "/kill", "turtle2");

	//Circle celebration dance
	rate.sleep();
	spawn_turtle(n, "/spawn", 10.5, 5.5, 1.57, "turtle2");
	turtle_set_pen(n, "/turtle2/set_pen", 50, 250, 50, 5, true);
	spawn_turtle(n, "/spawn", 0.5, 5.5, 4.71, "turtle3");
	turtle_set_pen(n, "/turtle3/set_pen", 50, 250, 50, 5, true);
	spawn_turtle(n, "/spawn", 6.5, 8, 4.71, "turtle4");
	turtle_set_pen(n, "/turtle4/set_pen", 50, 250, 50, 5, true);
	spawn_turtle(n, "/spawn", 4.5, 2, 1.57, "turtle5");
	turtle_set_pen(n, "/turtle5/set_pen", 50, 250, 50, 5, true);
	while(ros::ok())
	{
		rate.sleep();
		move_turtle(n, "/turtle2/cmd_vel", rate, 7.85, 0, 0, 0, 0, 1.57);		
		move_turtle(n, "/turtle3/cmd_vel", rate, 7.85, 0, 0, 0, 0, 1.57);
		move_turtle(n, "/turtle4/cmd_vel", rate, 3.14, 0, 0, 0, 0, -3.14);
		move_turtle(n, "/turtle5/cmd_vel", rate, 3.14, 0, 0, 0, 0, -3.14);
		ros::spinOnce();
	}
	kill_turtle(n, "/kill", "turtle2");
	kill_turtle(n, "/kill", "turtle3");
	kill_turtle(n, "/kill", "turtle4");
	kill_turtle(n, "/kill", "turtle5");
	return 0;
}
