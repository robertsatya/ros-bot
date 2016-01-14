#include <ros/ros.h>
#include "test_assignment_1/motion_node.h"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>


using namespace std;

ros::Publisher marker_pub;
ros::Publisher location_pub;
visualization_msgs::Marker marker;
geometry_msgs::Pose p;

bool select_mode(test_assignment_1::motion_node::Request  &req, test_assignment_1::motion_node::Response &res);

int main(int argc, char **argv){
	ros::init(argc, argv, "odroid_node");
	ros::NodeHandle n,n1,n2;
	ros::Rate r(1);
 
	p.position.x = 0;
  p.position.y = 0;
  p.position.z = 0;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 1.0;

	ros::ServiceServer service = n.advertiseService("motion_node_service",select_mode);
	location_pub = n1.advertise<geometry_msgs::PoseStamped>("loc", 100);
  marker_pub = n2.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "motion_node";
    marker.id = 0;

	  marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker.pose.position = p.position;
		marker.pose.orientation = p.orientation;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    marker_pub.publish(marker);
		
		r.sleep();
	ros::spin();

	return 0;
}

bool select_mode(test_assignment_1::motion_node::Request  &req, test_assignment_1::motion_node::Response &res)
{

	ros::NodeHandle n,n1;
	res.mode = req.mode;
	int dir = res.mode;
	ROS_INFO("\n Selected mode is %d", dir);
  ros::Rate r(1);

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	location_pub = n1.advertise<geometry_msgs::PoseStamped>("loc", 100);

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		float fb=0,lr=0;
		if(dir==1)
			fb = 0.5;
		else if(dir==2)
			fb = -0.5;
		else if(dir==3)
			lr = 0.5;
		else if(dir==4)
			lr = -0.5;
		else return 0;

	  p.position.x += fb;
    p.position.y += lr;
  
    marker.pose.position = p.position;
		marker.pose.orientation = p.orientation;

		ROS_INFO(" %lf %lf \n",marker.pose.position.x,marker.pose.position.y);
    // Publish the marker
    marker_pub.publish(marker);
		
		geometry_msgs::PoseStamped ps;
		ps.header.stamp = ros::Time::now();
		ps.header.frame_id = "/my_frame";
		ps.pose = p;
		location_pub.publish(ps);

	if (res.mode == 0) {
		exit(0);
	}
	return true;
}
