#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <stdexcept>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <control_node/states.h>

using namespace std;

int control_state;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "central_control");
	control_state = STATE_INIT;
	while(control_state == STATE_INIT) {
		continue;
	}
	cout << control_state << endl;
	return 0;
}