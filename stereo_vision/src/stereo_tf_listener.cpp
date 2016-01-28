#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
ros::Publisher location_pub;
visualization_msgs::Marker marker;
geometry_msgs::Pose p;

void locCallback(const geometry_msgs::PointStamped &loc)
{
  tf::TransformListener listener;

	ros::NodeHandle node;
  ros::Rate rate(1.0);
    tf::StampedTransform transform;
//		const ros::Time time = ros::Time(0);

		geometry_msgs::PoseStamped parent_point;
		geometry_msgs::PoseStamped loc1;
    loc1.pose.orientation.x = 0;
    loc1.pose.orientation.y = 0;
    loc1.pose.orientation.z = 0;
    loc1.pose.orientation.w = 0;
		loc1.pose.position.x = loc.point.z;
		loc1.pose.position.y = loc.point.x;
		loc1.pose.position.z = loc.point.y;
    loc1.header = loc.header;

		parent_point = loc1;
    try{
			listener.waitForTransform("/stereo_cam", "/usb_cam",
                              ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/stereo_cam", "/usb_cam",
                               ros::Time(0), transform);
			geometry_msgs::PoseStamped child_point;
			child_point = loc1;

	  	child_point.header.stamp = ros::Time();

			ROS_INFO("\nTransform %lf %lf %lf",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    	listener.transformPose("usb_cam", child_point, parent_point);


    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    //  continue;
    }
  	ros::NodeHandle n;


    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
			marker.header = parent_point.header;
      marker.header.stamp = ros::Time::now();
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose = parent_point.pose;
  		ROS_INFO(" %lf %lf %lf \n",marker.pose.position.x,marker.pose.position.y,marker.pose.position.z);
      // Publish the marker
      marker_pub.publish(marker);

 /*   try{
      listener1.lookupTransform("/usb_cam", "/world",
                               ros::Time(0), transform1);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }*/
/*    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                    transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
*/
    rate.sleep();
};

int main(int argc, char** argv){
  ros::init(argc, argv, "stereo_tf_listener");

  ros::NodeHandle n,n1;

	ros::Subscriber sub = n.subscribe("/left_point", 10, locCallback);
	p.position.x = 0;
  p.position.y = 0;
  p.position.z = 0;
  p.orientation.x = 0.0;
  p.orientation.y = 0.0;
  p.orientation.z = 0.0;
  p.orientation.w = 1.0;

  marker_pub = n1.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;

    marker.header.frame_id = "/usb_cam";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "ball";
    marker.id = 0;

	  marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header

    marker.pose.position = p.position;
		marker.pose.orientation = p.orientation;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.11;

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

	ros::spin();
	return 0;
}


