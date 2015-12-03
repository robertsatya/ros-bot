#include <ros/ros.h>
#include<geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <joy_test/JoyIn.h>

using namespace std;
int x_mid = 320;
int thresh = 1;
float fwd_thresh = 1;


void reach_callback(joy_test::JoyIn::Request  &req, joy_test::JoyIn::Response &res)
{

//  	vel.linear.x = ax[1];
//		vel.angular.z = atan2(ax[2],(ax[3]));

	ros::NodeHandle n;
	ros::service::waitForService("joy_in");
  ros::ServiceClient client = n.serviceClient<joy_test::JoyIn>("joy_in");
  joy_test::JoyIn srv;
	char h=65,l=65;
	long int vel = 0;
	int max_vel = 200;
	int diff = req.x - x_mid;
	int thresh = 1;
	vector<uint8_t> stream;
	{
		if(diff<(-thresh))
		{
			cout << "Rotating left\n";
			stream.push_back(145);
			stream.push_back(0);
			stream.push_back(150);
			stream.push_back(255);
			stream.push_back(106);
			srv.request.stream = stream;
		  if (client.call(srv))
  		{
    		ROS_INFO("Success: %d", srv.response.success);
  		}
  		else
  		{
    		ROS_ERROR("Failed to call service joy_in");
	    	return;
  		}
		}
		else if(diff>thresh)
		{
			cout << "Rotating right\n";
			stream.push_back(145);
			stream.push_back(255);
			stream.push_back(106);
			stream.push_back(0);
			stream.push_back(150);
			srv.request.stream = stream;
		  if (client.call(srv))
  		{
    		ROS_INFO("Success: %d", srv.response.success);
  		}
  		else
  		{
    		ROS_ERROR("Failed to call service joy_in");
	    	return;
  		}
		}
		else  
		{
			vel = (long int)(max_vel);
			cout << "Moving at " << vel << " mm/s\n";
			l = (vel&0x00ff);
			h = ((vel>>8)&0x00ff);
			stream.push_back(145);
			stream.push_back(h);
			stream.push_back(l);
			stream.push_back(h);
			stream.push_back(l);
			srv.request.stream = stream;
	  	if (client.call(srv))
			{
	  		ROS_INFO("Success: %d", srv.response.success);
			}
			else
			{
  			ROS_ERROR("Failed to call service joy_in");
		  	return;
			}
			
			int safe = 1;
			ros::NodeHandle dist_n;
		  ros::ServiceClient client_dist = dist_n.serviceClient<joy_test::JoyIn>("joy_in");
			joy_test::MinDistThresh srv_dist;
			srv_dist.request.thresh = fwd_thresh;

			while(safe)
			{
		  	if (client.call(srv))
				{
					safe = srv.response.safe;
		  		ROS_INFO("Safe: %d", safe);
				}
				else
				{
	  			ROS_ERROR("Failed to call service joy_in");
			  	return;
				}
			}
		}
	}
//  srv.request.b = atoll(argv[2]);
//sleep(1);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "reach_target_node");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("reach_target",reach_callback);
	ros::spin();
	return 0;
}


