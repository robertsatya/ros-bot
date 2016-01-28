#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

using namespace std;

ros::Publisher location_pub;
geometry_msgs::Pose p;
std::string frame_name;

int main(int argc, char **argv){
	ros::init(argc, argv, "dummy_loc");
	ros::NodeHandle n;
	ros::Rate r(100);
		 
//  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
//  frame_name = argv[1];
//	frame_name = "stereo_cam";

	location_pub = n.advertise<geometry_msgs::PoseStamped>("loc", 100);
 	
//	tf::TransformBroadcaster br;
	while(n.ok())
	{

		ROS_INFO("Pos X Y:\n");
		cin >> p.position.x >> p.position.y >> p.position.z ;	
//  	p.position.x = 0;
//    p.position.y = 0;
//    p.position.z = 0;
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = 0.0;
    p.orientation.w = 1.0;

		geometry_msgs::PoseStamped ps;
		ps.header.stamp = ros::Time::now();
		ps.header.frame_id = "/stereo_cam";
		ps.pose = p;
		location_pub.publish(ps);

  	p.position.x = 1;
    p.position.y = 1;
 
//		r.sleep();
		
		ros::spinOnce();

/*  	tf::Transform transform;
  	transform.setOrigin( tf::Vector3(p.position.x, p.position.y, 0.0) );
  	tf::Quaternion q;
  	q.setRPY(0, 0, p.orientation.z);
  	transform.setRotation(q);
		ROS_INFO("\nTransform before%lf %lf",transform.getOrigin().x(),transform.getOrigin().y());*/
/*	br.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"usb_cam", "stereo_cam"));	*/
//  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "usb_cam", frame_name));
//		ROS_INFO("\nTransform after%lf %lf",transform.getOrigin().x(),transform.getOrigin().y());
//		ros::Duration(100).sleep();
//			r.sleep();
	}

	return 0;
}
