#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rob_nav/navigationAction.h>
#include <geometry_msgs/PoseStamped.h>

using namespace rob_nav;
typedef actionlib::SimpleActionClient<navigationAction> Client;

class MyNode
{
  ros::NodeHandle n;
public:

	uint8_t fin;
  MyNode() : ac("nav_act", true)
  {
		fin = 0;
    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();
    ROS_INFO("Action server started, sending goal.");
  }
	


  void doStuff(geometry_msgs::PointStamped point, int mode, double angle, int dir, int cmd_freq)
  {
    navigationGoal goal;
    goal.dest = point;
		goal.type = mode;
		goal.angle = angle;
		goal.dir = dir;
		goal.cmd_freq = cmd_freq;

		fin = 0;
    // Need boost::bind to pass in the 'this' pointer
    ac.sendGoal(goal,
                boost::bind(&MyNode::doneCb, this, _1, _2),
                boost::bind(&MyNode::activeCb, this),
                boost::bind(&MyNode::feedbackCb, this, _1));

  }

	// Called once when the goal completes
	void doneCb(const actionlib::SimpleClientGoalState& state,
	            const navigationResultConstPtr& result)
	{
	  ROS_INFO("Got Final location : (%f , %f , %f, %f)\n", result->final_loc.point.x,result->final_loc.point.y,result->final_loc.point.z,result->angle);
/*	  ROS_INFO("Finished in state [%s]", state.toString().c_str());
	  ROS_INFO("Answer: %i", result->sequence.back());*/
		fin = 3;
//	  ros::shutdown();
	}
	
	// Called once when the goal becomes active
	void activeCb()
	{
	  ROS_INFO("Goal just went active");
		fin =1;
	}
	
	// Called every time feedback is received for the goal
	void feedbackCb(const navigationFeedbackConstPtr& feedback)
	{
		
	  ROS_INFO("Got Feedback of location : (%f , %f , %f)\n", feedback->cur_loc.point.x,feedback->cur_loc.point.y,feedback->cur_loc.point.z);

/*		if(feedback->sequence.size()==10)
		{
			fin=1;
			ac.cancelGoal();	
		}*/
		fin=2;
	}

private:
  Client ac;
};


