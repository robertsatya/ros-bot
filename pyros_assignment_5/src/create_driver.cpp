#include <iostream>
#include <cstdio>
#include <ros/ros.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <malloc.h>

#define ROTATIONCHANGE 300
#define VELOCITYCHANGE 200

int prev_cmd = 0;
int curr_cmd = 0;
int fd = -1;

using namespace std;

int open_port(void);
int configure_port(int fd);
bool control_callback(pyros_assignment_5::create_driver::Request  &req, pyros_assignment_5::create_driver::Response &res);


int main(int argc, char const *argv[])
{
	ros::init(argc, argv, "create_driver");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("create_control_service",control_callback);

	fd = open_port();
	configure_port(fd);

	ros::spin();

	close(fd);
	return 0;
}

int open_port(void)
{
	int fd;
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);

	if(fd == -1) {
		printf("open_port: Unable to open /dev/ttyUSB0. \n");
	} else {
		fcntl(fd, F_SETFL, 0);
		printf("port is open.\n");
	}
	return(fd);
}

int configure_port(int fd)
{
	struct termios port_settings;

	cfsetispeed(&port_settings, B115200);
	cfsetospeed(&port_settings, B115200);

	port_settings.c_cflag &= ~PARENB;		// set no parity
	port_settings.c_cflag &= ~CSTOPB;		// set parity bits to 1
	port_settings.c_cflag &= ~CRTSCTS;		// disable hardware flow control
	port_settings.c_cflag &= ~CSIZE;		// set databits to 8
	port_settings.c_cflag |= CS8;

	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control

	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return(fd);

}

bool control_callback(pyros_assignment_5::create_driver::Request  &req, pyros_assignment_5::create_driver::Response &res)
{
	res.mode = req.mode;
	prev_cmd = curr_cmd;
	curr_cmd = res.mode;
	if (curr_cmd == prev_cmd)
	{
		return true;
	}
	size_t space = 1;
	unsigned char *byte = (unsigned char*)malloc(space);
	switch(curr_cmd) {
		case 'p':
			cout << "Power on\n";
			*byte = {128};
			break;
		case 's':
			cout << "Safe mode\n";
			*byte = {131};
			break;
		case 'f':
			cout << "Moving forward at 500 mm/s\n";
			byte = (unsigned char*)realloc(byte,5*space);
			byte[] = {145, 1, 255, 1, 255};
			break;
		case 'b':
			cout << "Brake\n";
			//unsigned char byte[] = {145, 0, 0, 0, 0};
			break;
		default:
			cout << "I dunno what that is\n";
			//unsigned char byte[] = {145, 0, 0, 0, 0};
			break;
	}
	//unsigned char byte[] = {128, 131, 145, 1, 255, 1, 255};
	write(fd, byte, sizeof(byte));

	return true;
}