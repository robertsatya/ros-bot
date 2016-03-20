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
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>

#include <control_node/states.h>
#include <control_node/BroadSearch.h>

#include <geometry_msgs/PointStamped.h>
#include <rob_nav/MyNode.hpp>
#include <rob_nav/tcp_client.h>

using namespace std;
using namespace cv;

int  prev_state;
std::vector<string> state_labels;

static bool isPointFound = false, isBucketFound = false;
static float x_pos=0, y_pos=0, depth_pos=0;
static float gposx = 0, gposy = 0, gposz = 0;

void sigHandle(int sig_id);

void locCallback(const geometry_msgs::PointStamped &loc);
void buckCallback(const geometry_msgs::PointStamped &loc);

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "central_control");
	control_state = STATE_INIT;
	signal(SIGINT, sigHandle);
	state_labels.push_back("STATE_ERROR");
	state_labels.push_back("STATE_INIT");
	state_labels.push_back("STATE_BROAD_SEARCH");
	state_labels.push_back("STATE_MOVE_TO_BALL");
	state_labels.push_back("STATE_ROTATE_SEARCH");
	state_labels.push_back("STATE_MOVE_TO_SEARCH");
	state_labels.push_back("STATE_CHECK_BALL_AT_TARGET");
	state_labels.push_back("STATE_REFINE_POSITION");
	state_labels.push_back("STATE_BALL_READY_TO_PICK");
	state_labels.push_back("STATE_GRAB_BALL");
	state_labels.push_back("STATE_LOCATE_GOAL");
	state_labels.push_back("STATE_MOVE_TO_GOAL");
	state_labels.push_back("STATE_DROP_BALL_AT_GOAL");
	state_labels.push_back("STATE_SEARCH_MISSED_BALLS");
	ros::NodeHandle n;
	// printf("Waiting for BroadSearchService\n");
	// ros::service::waitForService("broad_search_service",-1);
	// ros::ServiceClient broadSearchClient = n.serviceClient<control_node::BroadSearch>("broad_search_service", true);
	ros::Subscriber sub = n.subscribe("/left_point", 10, locCallback);
	ros::Subscriber gsub = n.subscribe("/buck_point", 10, buckCallback);
	control_node::BroadSearch searchsrv;
	float pos[3] = {0};
	MyNode motion_node;
	int frame_seq = 0;

	// MOVE related
	ros::AsyncSpinner spinner(4);
	spinner.start();
	int mode=1,dir=0,cmd_freq=1;
	double angle=0;
	geometry_msgs::PointStamped p;
	int m_success = 1;

	// BROAD_SEARCH related
	int broad_search_rotate_angle = 0;
	int angle_step = 45;

	// R_PI socket related
	tcp_client c;
	string host="192.168.43.97";
	c.conn(host , 9996);
	string o_str = "";
	string res = "";
	int count_5 = 0;
		int broad_3 = 0;

		int key = 0;
	// TODO: Try postion tracking while moving for simple mapping
	while(control_state != STATE_ERROR) {
		cout << "Current State : " << control_state << " - " << state_labels[control_state] << endl;
		switch(control_state) {
			case STATE_INIT:
				control_state = STATE_BROAD_SEARCH;
				cout << "State init" << endl;
//				cin >> key;
				break;
			case STATE_BROAD_SEARCH:
				if(broad_search_rotate_angle<360)
				{
					//spinner.start();
					//cout << "waiting here" << endl;
					sleep(5);
					cout << isPointFound << endl;
					int64 t = getTickCount();
					if (isPointFound)
					{
						pos[0] = x_pos;
						pos[1] = y_pos;
						pos[2] = depth_pos;
						// spinner.stop();
						printf("Received %f %f %f\n", pos[0], pos[1], pos[2]);
						t = getTickCount() - t;
						printf("Time elapsed: %fms\n", t*1000/getTickFrequency());
						//break;
						if(pos[2] > 0) {
							control_state = STATE_MOVE_TO_BALL;
							broad_search_rotate_angle = 0;
						} else if (broad_3 < 3) {
							control_state = STATE_BROAD_SEARCH;
							broad_3++;
						} else {
							printf("Rotating to search again");
							control_state = STATE_ROTATE_SEARCH;
							broad_3 = 0;
						}
					}
				}
				else
				{
					broad_search_rotate_angle = 0;
					control_state = STATE_MOVE_TO_SEARCH;
					printf("Moving to search again");
				}

//				cin >> key;

				break;
			case STATE_ROTATE_SEARCH:
				broad_search_rotate_angle += angle_step;
				mode = 0;
				angle = 20;
				m_success = 1;
/*				cout << "Mode" << mode << "Angle" << angle << endl;
				cin >> key;*/
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin < 3)
				{
				}
				control_state = STATE_BROAD_SEARCH;
				//cin >> key;

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
				m_success = 1;
/*				cout << "Pos:" << p.point.x << " " << p.point.y << " Mode:" << mode << endl;  
				cin >> key;*/
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin<3)
				{
				}
				
				if(m_success == 2)
				{
					cout << control_state << endl;
					exit(0);
				}
				control_state = STATE_BROAD_SEARCH;

//				cin >> key;

				break;
			case STATE_MOVE_TO_BALL:
				m_success = 1;
				cout << "Going to ball" << endl;
				p.header.seq = frame_seq++;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "/robot";
				p.point.x = pos[2];
				p.point.y = -pos[0];
				p.point.z = -pos[1];
				mode = 1;
/*				cout << "Pos:" << p.point.x << " " << p.point.y << " Mode:" << mode << endl;  
				cin >> key;*/
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin<3)
				{
				}
				if(m_success == 2)
				{
					cout << control_state << endl;
					exit(0);
				}
				if(m_success == 1)
					control_state = STATE_REFINE_POSITION;
				else
					control_state = STATE_BROAD_SEARCH;

	//			cin >> key;

//				control_state = STATE_ERROR; // TODO: Move to different states to block
				break;
			case STATE_REFINE_POSITION:
/*				cout << "Mode: 5" << endl; 
				cin >> key;*/
			motion_node.doStuff(p,5,angle,dir,cmd_freq,m_success);
							while(motion_node.fin<3)
							{
							}
				sleep(5);
				o_str = "1";
				mode = 2;
				count_5 = 0;
	//			cin >> key;
				while(true)
				{
					// TODO: Ignore first 2-3 commands
					// receive and echo reply
					// cout << c.receive(1024);
					c.send_data(o_str);
					cout << "sent 1 to rpi" << endl;
					res = c.receive(3);
					cout << res << endl;
					dir = boost::lexical_cast<int>(res[0]);
					cmd_freq = boost::lexical_cast<int>(res[2]);

					// res.angle = boost::lexical_cast<float>(c.receive(1024));
					cout << dir << endl;
					if(dir==4 || (dir == 5 && count_5 == 1))
						break;
					if(count_5 == 1)
						count_5 = 0;
					if(dir == 5)
						count_5++;
					else
					{
						m_success = 1;
/*						cout << "Mode: " << mode << " Dir: " << dir << " Cmd_freq: " << cmd_freq << endl;
						cin >> key;*/
						motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
						if(motion_node.fin<3)
						{
						}
					}
				}

				if(dir == 4)
				{
					control_state = STATE_GRAB_BALL;
					res = c.receive(3);
				}
				else
				{
					control_state = STATE_BROAD_SEARCH;
				}

//				exit(0);
//				cin >> key;

				//TODO: Turn around to see if ball in vicinity
				break;
			case STATE_BALL_READY_TO_PICK:
/*				o_str = "2";
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
				}*/
//				cin >> key;

				break;
			case STATE_GRAB_BALL:
/*				mode = 3;
				p.header.seq = frame_seq++;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "/robot";
				p.point.x = 0;
				p.point.y = 0;
				p.point.z = 0;
				m_success = 1;
//				cout << "Mode:" << mode << "Pos: 0 0" << endl;
//				cin >> key;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin<3)
				{
				}
				if(m_success == 2)
				{
					cout << control_state << endl;
					exit(0);
				}*/
				control_state = STATE_LOCATE_GOAL;
			
//				cin >> key;

				break;
			case STATE_LOCATE_GOAL:
				/*spinner.start();
				sleep(2);
				if (isBucketFound) {
					p.header.seq = frame_seq++;
					p.header.stamp = ros::Time::now();
					p.header.frame_id = "/robot";
					p.point.x = gposx;
					p.point.y = gposy;
					p.point.z = gposz;
					spinner.stop();
					mode = 1;
					motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
					while(motion_node.fin<3)
					{
					}
					control_state = STATE_REFINE_GOAL_POS;
				}*/
				control_state = STATE_REFINE_GOAL_POS;
//				cin >> key;

				break;
			case STATE_REFINE_GOAL_POS:
				o_str = "3";
				mode = 2;
				count_5 = 0;
				while(true)
				{
					// TODO: Ignore first 2-3 commands
					// receive and echo reply
					// cout << c.receive(1024);
					c.send_data(o_str);
					cout << "sent 3 to rpi" << endl;
					res = c.receive(3);
					cout << res << endl;
					dir = boost::lexical_cast<int>(res[0]);
					cmd_freq = boost::lexical_cast<int>(res[2]);

					// res.angle = boost::lexical_cast<float>(c.receive(1024));
					cout << dir << endl;
					if(dir==4 || (dir == 5 && count_5 == 1))
						break;
					if(count_5 == 1)
						count_5 = 0;
					if(dir == 5)
						count_5++;
					else
					{

						m_success = 1;
						if(dir == 8)
						{
							/*cout << "Mode" << mode << "Angle" << angle << endl;
							cin >> key;*/
							motion_node.doStuff(p,0,30,dir,cmd_freq,m_success);
							while(motion_node.fin<3)
							{
							}
							sleep(1);
						}
						else
						{
						/*	cout << "Mode: " << mode << " Dir: " << dir << " Cmd_freq: " << cmd_freq << endl;
							cin >> key;*/
							motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
							while(motion_node.fin<3)
							{
							}
						}
					}
				}
							cout << "Mode: 4" << endl;
							cin >> key;
							motion_node.doStuff(p,4,angle,dir,cmd_freq,m_success);
							while(motion_node.fin<3)
							{
							}

				control_state = STATE_BROAD_SEARCH;;
//				cin >> key;

				break;
			case STATE_DROP_BALL_AT_GOAL:
/*				o_str = "3";
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
				}*/
//				cin >> key;

				break;
			case STATE_SEARCH_MISSED_BALLS:
				mode = 1;
				p.header.seq = frame_seq++;
				p.header.stamp = ros::Time::now();
				p.header.frame_id = "/robot";
				p.point.x = -30;
				p.point.y = 0;
				p.point.z = 0;
				m_success = 1;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin<3)
				{
				}
				mode = 0;
				angle = 30;
				m_success = 1;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin<3)
				{
				}
				mode = 0;
				angle = -60;
				m_success = 1;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin<3)
				{
				}
				mode = 0;
				angle = 30;
				m_success = 1;
				motion_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
				while(motion_node.fin<3)
				{
				}
//				cin >> key;

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

void locCallback(const geometry_msgs::PointStamped &loc) {

	x_pos = loc.point.x;
	y_pos = loc.point.y;
	depth_pos = loc.point.z;
	isPointFound = true;

//	printf("Point Published : %f %f %f\n", x_pos, y_pos, depth_pos);
}

void buckCallback(const geometry_msgs::PointStamped &loc) {

	gposx = loc.point.x;
	gposy = loc.point.y;
	gposz = loc.point.z;
	isBucketFound = true;

//	printf("Point Published : %f %f %f\n", x_pos, y_pos, depth_pos);
}



