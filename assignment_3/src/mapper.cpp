#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/SetPen.h"
#include "turtlesim/Spawn.h"
#include <sstream>
#include <iostream>

using namespace std;

int flag = 0;
const double PI = 3.14159265359;
ros::Subscriber scanSub;
ros::Subscriber odomSub;
ros::Publisher velPub;

void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double relative_angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void poseCallback(const geometry_msgs::Point::ConstPtr & pose_message);
float minIn(int start, int end, std::vector<float> v);


void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	ros::Rate loop(1);

	std::vector<float> scanRange = scan->ranges;
	std::cout << "yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy" << endl;

	float val620_660 = minIn(560, 720, scanRange);
	float val320_640 = minIn(320, 640, scanRange);
	float val640_960 = minIn(640, 960, scanRange);

	if (flag==0) {
		cout << scanRange.size() << endl;
		for (int i = 0; i < scanRange.size(); ++i)
		{
			std::cout << scanRange[i] << " " ;
		}
		flag = 1;
	}
	int dir = rand()%2 ? 1 : -1;


	if (val620_660 > 0.5 && val620_660 < 0.75) {
		if (val640_960 > 0.5 && val320_640 < 0.5) {
			//rotate(degrees2radians(45),degrees2radians(45),0);
			 vel_msg.angular.z = degrees2radians(145);
			 cout << "Rotating +45 : " << val640_960 << endl;
		} else if (val640_960 < 0.5 && val320_640 > 0.5) {
			//rotate(degrees2radians(45),degrees2radians(45),1);
			 vel_msg.angular.z = degrees2radians(-145);
			 cout << "Rotating -45 : " << val320_640 << endl;
		} else if (val640_960 > 0.5 && val320_640 > 0.5) {
			rotate(degrees2radians(1345),degrees2radians(145),rand()%2);
			// vel_msg.angular.z = degrees2radians(dir*45);
			// cout << "Rotating random : "  << val320_640 << " : " << val620_660 << " : " << val640_960 << endl;
		} else {
			vel_msg.linear.x = -18.9;
			cout << "Moving back : " << val320_640  << " : " << val620_660 << " : " << val640_960 << endl;
		}
	} else if (val620_660 <= 0.5) {
		vel_msg.linear.x = -18.9;
		cout << "Moving back : " << val320_640  << " : " << val620_660 << " : " << val640_960 << endl;
	} else {
		vel_msg.linear.x = 14.9;
		cout << "Moving forward : " << val620_660 << endl;
	}








	// if (val620_660 >= 0.7) {
	// 	vel_msg.linear.x = 0.9;
	// 	cout << "Moving forward : " << val620_660 << endl;
	// }


	// else if((val620_660 > 0.5 && val620_660 < 0.75) && (val320_640 > 0.5) && (val640_960 > 0.5)){
	// 	vel_msg.angular.z = degrees2radians(dir*45);
	// 	cout << "Rotating random : "  << val320_640 << " : " << val620_660 << " : " << val640_960 << endl;
	// }


	// else if ((val620_660 > 0.5 && val620_660 < 0.75) && (val640_960 > 0.5 && val640_960 < 0.75)) {
	// 	vel_msg.angular.z = degrees2radians(45);
	// 	cout << "Rotating -45 : " << val640_960 << endl;
	// }


	// else if ((val620_660 > 0.5 && val620_660 < 0.75) && (val320_640 > 0.5 && val320_640 < 0.75)) {
	// 	vel_msg.angular.z = degrees2radians(-45);
	// 	cout << "Rotating +45 : " << val320_640 << endl;
	// }


	// else {
	// 	vel_msg.linear.x = -0.9;
	// 	cout << "Moving back : " << val320_640  << " : " << val620_660 << " : " << val640_960 << endl;
	// }
	velPub.publish(vel_msg);
	//loop.sleep();
	//ros::spinOnce();


	// if(scanRange[640] > 0.01 && scanRange[640] < 0.05) {
	// 	rotate(degrees2radians(45), degrees2radians(45), 1);
	// } else if (scanRange[640] >= 0.01) {
	// 	move(0.5, 0.01, 1);
	// } else {
	// 	move(0.5, 0.01, 0);
	// }

}

float minIn(int start, int end, std::vector<float> v) {
	float max = 999.00;
	for (int i = start; i < end; ++i)
	{
		max = max > v[i] ? v[i] : max;
	}
	return max;
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "mapper");

	ros::NodeHandle n;
	scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, &processLaserScan);
	//odomSub = n.subscribe<nav_msgs::Odometry>("/odom", 100, &poseCallback);
	velPub = n.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1000);
	ros::Rate loop_rate(0.1);

	loop_rate.sleep();
	while (1) {
		ros::spinOnce();
	}
	return 0;
}

void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;

	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(10);
	do{
		velPub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		if (current_distance>=distance) {
			break;
		}
		//ros::spinOnce();
		loop_rate.sleep();

	} while(current_distance<distance);
	vel_msg.linear.x =0;
	velPub.publish(vel_msg);
}

void rotate (double angular_speed, double relative_angle, bool clockwise){
	geometry_msgs::Twist vel_msg;

	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;

	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(5);
	do{
		velPub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		if(current_angle > relative_angle) {
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
		cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	} while(current_angle<relative_angle);
	vel_msg.angular.z =0;
	velPub.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

// void poseCallback(const nav_msgs::Odometry::ConstPtr & pose_message){
// 	geometry_msgs::PoseWithCovariance position;
// 	position = pose_message.pose;
// }