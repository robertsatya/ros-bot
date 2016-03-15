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
	int m_success = 1;

/*    tcp_client c;

//    string host="192.168.0.17";
    string host="192.168.43.97";
//    string host="192.168.43.129";

         
    //connect to host
    c.conn(host , 9998);
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
							cout << "\nPlease enter cmd_freq";
							cin >> cmd_freq;
							break;
			case 3: cout << "\nPlease enter forward absolute X coordinate";
				  		cin >> p.point.x;
							cout << "\nPlease enter forward absolute Y coordinate";
				  		cin >> p.point.y;
			default: break;
		}
	
  	p.point.z = 0;

/*		
    //send some data
//    string o_str = dir_str + " " + boost::lexical_cast<std::string>(angle);
		string o_str = "1";
    c.send_data(o_str);

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

// TODO: add counter for dir 5
		if(dir==4 || dir == 5)
		{
			cout << "dir" << dir;
			if(dir==4)
			{	
				cout << "before sending ;";
				sleep(1);
				o_str = "3";
				cout << "data prepared ;";
				sleep(1);
  			 c.send_data(o_str);
				cout << "data sent ;";
				sleep(1);
			
			string res1 = c.receive(1024);
				cout << "data received ;";
				sleep(1);
			cout << res1 << endl;
//				while(true)
//				{}
			}
		
		}
//		my_node.fin = 3;
*/
		m_success = 1;
   my_node.doStuff(p,mode,angle,dir,cmd_freq,m_success);
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
