#include<ros/ros.h>
#include <termios.h>

using namespace std;
int tcdrain(int fildes);

class iRobot
{
	private:
	int fd = -1;

	public:
	void start()
	{
	}
	
	int open_port(void)
	{
		int fd;
		fd = open("/dev/ttyUSB0", O_RDWR);
	
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
		port_settings.c_cflag &= ~CSTOPB;		// set stop bits to 1
		port_settings.c_cflag &= ~CRTSCTS;		// disable hardware flow control
		port_settings.c_cflag &= ~CSIZE;		// set databits to 8
		port_settings.c_cflag |= CS8;
	
		port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control
	
		tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
		return(fd);
	}

	void executeCb(rob_nav::serial &seq)
	{
		const int8_t cnt = seq.count;
		char cmd[cnt] = seq.cmd;
		write(fd, cmd, sizeof(cmd));
		tcdrain(fd);	
	}	
};
