#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <rob_nav/MyNode.hpp>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rob_nav/navigationAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <rob_nav/tcp_client.h>

using namespace std;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "nav_client");

	MyNode my_node;
	geometry_msgs::PointStamped p;
	int i=1,mode=0,dir=0,cmd_freq=1;
	double angle=0;

/*    tcp_client c;

//    string host="192.168.0.17";
    string host="192.168.43.97";
         
    //connect to host
    c.conn(host , 9991);

         
    //send some data
//    string o_str = dir_str + " " + boost::lexical_cast<std::string>(angle);
		string o_str = "1";
    c.send_data(o_str);
*/
	while(ros::ok())
	{
  	p.header.seq = i;
  	p.header.stamp = ros::Time::now();
  	p.header.frame_id = "/robot";
		cout << "Please enter mode: 1) Forward 2) Circular";
  	cin >> mode;
		
		switch(mode)
		{
			case 0: cout << "\nPlease enter angle";
  						cin >> angle;
							break;
			case 1: cout << "\nPlease enter forward X coordinate";
				  		cin >> p.point.x;
							cout << "\nPlease enter forward Y coordinate";
				  		cin >> p.point.y;
							break;
			case 2: cout << "\nPlease enter timed dir";
				  		cin >> dir;
							cmd_freq = 1;
			case 3: cout << "\nPlease enter max search angle";
							break;
			default: break;
		}
	
  	p.point.z = 0;

/*		
		mode = 2;
           
    //receive and echo reply
    cout<<"----------------------------\n\n";
//    cout << c.receive(1024);
		string res = c.receive(1024);
		cout << res << endl;
    dir = boost::lexical_cast<int>(res[0]);
		cmd_freq = boost::lexical_cast<int>(res[2]);

//    res.angle = boost::lexical_cast<float>(c.receive(1024));
    cout << dir;
    cout<<"\n\n----------------------------\n\n";

		if(dir==4 || dir == 5)
			break;
		my_node.fin = 3;
*/	
   my_node.doStuff(p,mode,angle,dir,cmd_freq);
//		cin >> my_node.fin;
  	while(my_node.fin<3)
  	{
  	}
		i++;
	}
 // ros::spinOnce();
  //exit
  return 0;
}
