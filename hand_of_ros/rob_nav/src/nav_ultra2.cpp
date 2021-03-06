#include<ros/ros.h>
#include<rob_nav/tcp_client.h>
#include<std_msgs/String.h>

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"nav_ultra2");
	tcp_client c;
	string host="192.168.43.97";
	c.conn(host , 1117);
	ros::NodeHandle n;
	ros::Publisher ultra = n.advertise<std_msgs::String>("ultra2", 1);
	while(true)
	{
		c.send_data("5");

		std_msgs::String ultra_sts;
		ultra_sts.data = c.receive(4);
//		cout << "Ultra: " << ultra << endl;
		ultra.publish(ultra_sts);
		ros::spinOnce();
	}
	return 0;
}

