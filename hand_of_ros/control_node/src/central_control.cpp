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

#include <rob_nav/MyNode.hpp>

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
	while(control_state != STATE_ERROR) {
		switch(control_state) {
			case STATE_INIT:
				control_state = STATE_BROAD_SEARCH;
				cout << "State init" << endl;
				break;
			case STATE_BROAD_SEARCH:
				if (broadSearchClient.call(searchsrv))
				{
					pos[0] = searchsrv.response.x;
					pos[1] = searchsrv.response.y;
					pos[2] = searchsrv.response.depth;
					printf("Received %f %f %f\n", pos[0], pos[1], pos[2]);
					control_state = STATE_MOVE_TO_BALL;
				}
				break;
			case STATE_MOVE_TO_BALL:
				cout << "Going to ball" << endl;
				control_state = STATE_ERROR;
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