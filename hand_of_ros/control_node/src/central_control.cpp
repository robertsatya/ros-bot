#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <csignal>
#include <string>
#include <stdexcept>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <control_node/states.h>
#include <control_node/BroadSearch.h>

#include <geometry_msgs/PointStamped.h>
#include <rob_nav/MyNode.hpp>
#include <rob_nav/tcp_client.h>

using namespace std;

int control_state, prev_state;

void sigHandle(int sig_id);

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "central_control");
	control_state = STATE_INIT;
	signal(SIGINT, sigHandle);
	ros::NodeHandle n;
	ros::ServiceClient broadSearchClient = n.serviceClient<control_node::BroadSearch>("broad_search_service");
	control_node::BroadSearch searchsrv;
	float pos[3] = {0};
	MyNode motion_node;
	int frame_seq = 0;

	//MOVE related
	int mode=1,dir=0,cmd_freq=1;
	double angle=0;
	geometry_msgs::PointStamped p;

	//BROAD_SEARCH related
	int broad_search_rotate_angle = 0;
	int angle_step = 45;

	//R_PI socket related 
  tcp_client c;
  string host="192.168.43.97";
  c.conn(host , 9991);
	string o_str = "";
	string res = "";

//TODO: Try postion tracking while moving for simple mapping
	while(control_state != STATE_ERROR) {
		switch(control_state) {
			case STATE_INIT:
				control_state = STATE_BROAD_SEARCH;
				cout << "State init" << endl;
				break;
			case STATE_BROAD_SEARCH:
				if(broad_search_rotate_angle<360)
				{
					if (broadSearchClient.call(searchsrv))
					{
						pos[0] = searchsrv.response.x;
						pos[1] = searchsrv.response.y;
						pos[2] = searchsrv.response.depth;
						if(pos[2] < 0)
						{
							printf("Received %f %f %f\n", pos[0], pos[1], pos[2]);
							control_state = STATE_MOVE_TO_BALL;
							broad_search_rotate_angle = 0;
						}
						else
						{
							printf("Rotating to search again");
							control_state = STATE_ROTATE_SEARCH;
						}
					}
				}
				else
				{
					broad_search_rotate_angle = 0;
					control_state = STATE_MOVE_TO_SEARCH;	
					printf("Moving to search again");
				}
				break;
			case STATE_ROTATE_SEARCH:
				broad_search_rotate_angle += angle_step;
				mode = 0;
				angle = 45;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				while(motion_node.fin<3)
  			{
  			}
				control_state = STATE_BROAD_SEARCH;
				break;
			case STATE_MOVE_TO_SEARCH:
				pos[0] = 0;
				pos[1] = 0;
				pos[2] = 100;
			 	p.header.seq = frame_seq++;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "/robot";
				p.point.x = pos[2];
				p.point.y = -pos[0];
				p.point.z = -pos[1];
				mode = 1;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				while(motion_node.fin<3)
  			{
  			}
				control_state = STATE_BROAD_SEARCH;
				break;
			case STATE_MOVE_TO_BALL:
				cout << "Going to ball" << endl;
			 	p.header.seq = frame_seq++;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "/robot";
				p.point.x = pos[2];
				p.point.y = -pos[0];
				p.point.z = -pos[1];
				mode = 1;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				while(motion_node.fin<3)
  			{
  			}
				control_state = STATE_REFINE_POSITION;
				break;
			case STATE_REFINE_POSITION:
				o_str = "1";
				mode = 2;
		    c.send_data(o_str);

				while(true)
				{           
					//TODO: Ignore first 2-3 commands
			    //receive and echo reply
			//    cout << c.receive(1024);
					res = c.receive(1024);
					cout << res << endl;
			    dir = boost::lexical_cast<int>(res[0]);
					cmd_freq = boost::lexical_cast<int>(res[2]);
			
			//    res.angle = boost::lexical_cast<float>(c.receive(1024));
			    cout << dir << endl;
					if(dir==4 || dir == 5)
						break;

					motion_node.doStuff(p,mode,angle,dir,cmd_freq);
		
				}
				
				if(dir == 4)
				{
					control_state = STATE_BALL_READY_TO_PICK; 
				}
				else
				{
					control_state = STATE_BROAD_SEARCH;
				}
				break;
			case STATE_BALL_READY_TO_PICK:
				o_str = "2";
		    c.send_data(o_str);
				res = c.receive(1024);
				cout << res << endl;

				if(boost::lexical_cast<int>(res[0]) == 1)
				{           
					control_state = STATE_GRAB_BALL;
				}
				else
				{
					control_state = STATE_REFINE_POSITION;
				}
				break;
			case STATE_GRAB_BALL:
				mode = 3; 
			 	p.header.seq = frame_seq++;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "/robot";
				p.point.x = 0;
				p.point.y = 0;
				p.point.z = 0;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				while(motion_node.fin<3)
  			{
  			}
				control_state = STATE_DROP_BALL_AT_GOAL;
			case STATE_DROP_BALL_AT_GOAL:
				o_str = "3";
		    c.send_data(o_str);
				res = c.receive(1024);
				cout << res << endl;

				control_state = STATE_SEARCH_MISSED_BALLS;
				if(boost::lexical_cast<int>(res[0]) == 1)
				{           
					control_state = STATE_BROAD_SEARCH;
				}
				else
				{
					control_state = STATE_REFINE_POSITION;
				}
				break;
			case STATE_SEARCH_MISSED_BALLS:
				mode = 1; 
			 	p.header.seq = frame_seq++;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "/robot";
				p.point.x = -30;
				p.point.y = 0;
				p.point.z = 0;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				mode = 0; 
				angle = 30;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				mode = 0; 
				angle = -60;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				mode = 0; 
				angle = 30;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq);
				while(motion_node.fin<3)
  			{
  			}
					
				break;
		}
	}
	cout << control_state << endl;
	return 0;
}

void sigHandle(int sig_id) {
	cerr << "Error occured. Stopping state: " << control_state << endl;
	control_state = STATE_ERROR;
}
