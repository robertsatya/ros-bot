#include<serial/serial.h>
#include<ros/ros.h>
#include <unistd.h>
#include<iostream>
#include<cstdio>

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"nav_serial");
	serial::Serial serial_port_("/dev/ttyUSB0",115200,serial::Timeout::simpleTimeout(1000));

	ros::NodeHandle n;
	ros::Rate r(1);
//	while(1)
//	{
/*	try
	{ 
        serial_port_.setPort("/dev/ttyUSB0");
        serial_port_.setBaudrate(115200);
        serial_port_.open();
		 		printf("port is open.\n");

    }
	catch(std::exception& e){
        std::cerr<<"iRobotCreate2 Error opening serial port: "<<e.what()<<std::endl;
    }
*/
		uint8_t sense[2],byte[5],cmd[1];

		cout << serial_port_.isOpen() << " " << serial_port_.getParity() << " " << serial_port_.getStopbits() << " " << serial_port_.getFlowcontrol() << endl;
//		tcflush(fd,TCIOFLUSH);
		cmd[0] = 128;
//		write(fd, cmd, sizeof(cmd));
		size_t bytes_wrote = serial_port_.write(cmd, sizeof(cmd));

		cout << bytes_wrote << endl;
//		tcdrain(fd);
		r.sleep();

		cmd[0] = 131;
//		write(fd, cmd, sizeof(cmd));
		serial_port_.write(cmd, sizeof(cmd));

//		tcdrain(fd);
		r.sleep();

		int right_vel = 25, left_vel = 25;
	 		cout << "Moving forward at L:" << left_vel << " R:" << right_vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = right_vel;
  		byte[3] = 0;
  		byte[4] = left_vel;
		serial_port_.write(byte, 5);

		r.sleep();
			
		    try{ 
        serial_port_.close();}
    catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        }

//	}

	return 0;
}
