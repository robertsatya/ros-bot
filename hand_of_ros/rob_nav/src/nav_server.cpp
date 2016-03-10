#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <rob_nav/navigationAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <serial/serial.h>

#define PI 3.141592

using namespace std;

class navigationAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<rob_nav::navigationAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  rob_nav::navigationFeedback feedback_;
  rob_nav::navigationResult result_;
  geometry_msgs::PoseStamped stationary_loc;
	int fd;
	serial::Serial serial_port_;
	ros::Rate rw,r;
	uint8_t byte[5],sense[2],cmd[1];
	double dist_per_enc;
	double axle_length;
	int vel;
	int turn_dir; // 0-No turn, 1-Left, 2-Right
	int time_burst_sleep;
public:

  navigationAction(std::string name) :
    as_(nh_, name, boost::bind(&navigationAction::executeCB_mod, this, _1), false),
    action_name_(name),
		serial_port_("/dev/ttyUSB0",115200,serial::Timeout::simpleTimeout(1000)),
		rw(50),
		r(8),
		axle_length(235),
		vel(200),
		turn_dir(0),
		time_burst_sleep(31250)
  {
		stationary_loc.header.seq = 0;
  	stationary_loc.header.stamp = ros::Time::now();
  	stationary_loc.header.frame_id = "/robot";
  	stationary_loc.pose.position.x = 0;
  	stationary_loc.pose.position.y = 0;
  	stationary_loc.pose.position.z = 0;
  	stationary_loc.pose.orientation.x = 0;
  	stationary_loc.pose.orientation.y = 0;
  	stationary_loc.pose.orientation.z = 0;
  	stationary_loc.pose.orientation.w = 1;

    as_.start();
//		dist_per_enc = 0.4446; //72*PI/508.8;
		dist_per_enc = 0.003451*53/100; //72*PI/65536;

  }

  ~navigationAction(void)
  {
		try
		{ 
      serial_port_.close();
		}
    catch(std::exception& e)
		{
      std::cerr<<e.what()<<std::endl;
    }
  }

	void start()
	{
		cmd[0] = 128;
		serial_port_.write(cmd, sizeof(cmd));
		rw.sleep();
	}

	void safe()
	{
		cmd[0] = 131;
		serial_port_.write(cmd, sizeof(cmd));
		rw.sleep();
	}

	double get_distance(uint16_t right_enc, uint16_t left_enc)
	{
		double d = (right_enc+left_enc)*dist_per_enc/2;
		return d;
	}

	void move_straight(uint16_t right_vel,uint16_t left_vel)
	{
 		cout << "Moving forward at L:" << left_vel << " R:" << right_vel << " mm/s\n";
 		byte[0] = 145;
 		byte[1] = right_vel >> 8;
 		byte[2] = right_vel & 0x00ff;
 		byte[3] = left_vel >> 8;
 		byte[4] = left_vel & 0x00ff;
		serial_port_.write(byte, 5);
		rw.sleep();
	}

	void move_turn(uint16_t right_vel,uint16_t left_vel)
	{
 		cout << "Turning at L:" << left_vel << " R:" << right_vel << " mm/s\n";
//		uint16_t left_vel1 = (65535 - left_vel) + 1; 
 		byte[0] = 145;
 		byte[1] = right_vel >> 8;
 		byte[2] = right_vel & 0x00ff;
 		byte[3] = left_vel >> 8;
 		byte[4] = left_vel & 0x00ff;
		serial_port_.write(byte, 5);
		rw.sleep();
	}

/*	void move_right(long right_vel,long left_vel)
	{
		cout << "Moving forward at L:" << left_vel << " R:" << right_vel << " mm/s\n";
//		uint16_t right_vel1 = (65535 - right_vel) + 1; 
 		byte[0] = 145;
 		byte[1] = right_vel >> 8;
 		byte[2] = right_vel & 0x00ff;
 		byte[3] = left_vel >> 8;
 		byte[4] = left_vel & 0x00ff;
		serial_port_.write(byte, 5);
		rw.sleep();
	}*/

	void move_timed_straight(int dir, int freq=1)
	{
		for(int i=0; i<freq; i++)
		{
			move_straight(100*dir,100*dir);
			usleep(time_burst_sleep);
			move_straight(0,0);
			usleep(time_burst_sleep);
		}
	}

	void move_timed_turn(int dir, int freq=1)
	{
		for(int i=0; i<freq; i++)
		{
			move_turn(100*dir,-100*dir);
			usleep(time_burst_sleep);
			move_straight(0,0);
			usleep(time_burst_sleep);
		}
	}

	void move_controlled_straight(double dist, double &dist_final)
	{
		uint32_t sum_l=0,sum_r=0;
		uint16_t n=0,left=0,right=0;
		long diff_l=0,diff_r=0;

		n = read_left_enc();
		cout << "Left: " << n << endl; 
		left = n;
		rw.sleep();

		n = read_right_enc();
		cout << "Right: " << n << endl; 
		right = n;

		int vel_r=vel, vel_l=vel;
		long ddiff = 0;

		double dist_step = 0, angle_diff = 0, angle_diff_final = 0;
		cout << "Dist target:" << dist << "Dist final" << dist_final << endl;

		//dist_final = 0;
		int j = 0;
		//Coordinate in mm
		for(j=0; dist_final<=dist; j++)
		{

  		n = read_left_enc();
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			n = read_right_enc();
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 
			
			if(j>=3)
			{
  			ddiff += diff_l - diff_r;
  			int vel_change = ddiff/100;
				vel_change = (vel_change>0)?vel_change:-vel_change;
  			if(ddiff>50)
  			{
  				vel_r = vel + vel_change;
  				vel_l = vel;
  			}
  			else if(ddiff<-50)
  			{
  				vel_l = vel + vel_change;
  				vel_r = vel;
  			}
  			else
  			{
  				vel_l = vel;
					vel_r = vel;
  			}
  			cout << "Diff L:" << diff_l << " R:" << diff_r << " Diff diff:" << ddiff << endl;
			}
			else
			{
				ddiff = 0;
				vel_r = 25;
				vel_l = 25;
			}
	
//		feedback_.cur_loc

			move_straight(vel_r,vel_l);

			dist_step = get_distance(diff_r,diff_l);
			angle_diff = (diff_r - diff_l)*dist_per_enc/axle_length;

			angle_diff_final += angle_diff;
//				double cos_term = dist_step*cos(angle_diff_final); 

			dist_final += dist_step;
			cout << "\nDist final:" << dist_final << " Dist step:" << dist_step << endl;
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << endl;
			r.sleep();
		}

			
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		if(j!=0)
			cout << "Left: " << sum_l/j << " Right: " << sum_r/j << endl;

	}
	
	void move_controlled_turn(double angle_diff_est, double &angle_diff_final)
	{

		uint32_t sum_l=0,sum_r=0;
		uint16_t n=0,left=0,right=0;
		long diff_l=0,diff_r=0;

		n = read_left_enc();
		cout << "Left: " << n << endl; 
		left = n;
		rw.sleep();

		n = read_right_enc();
		cout << "Right: " << n << endl; 
		right = n;

		int vel_r=vel, vel_l=vel;
		long ddiff = 0;

		double angle_diff = 0;

		if(angle_diff_est >= 0)
		{
			vel_r = 100;
			vel_l = -vel_r;
		}
		else
		{
			vel_l = 100;
			vel_r = -vel_l;
		}

		cout << "Angle target:" << angle_diff_est << "Angle final" << angle_diff_final << endl;
		//Coordinate in mm
		int j=0;
		for(j=0; !((angle_diff_final<(angle_diff_est+0.04)) && ((angle_diff_final) > (angle_diff_est-0.04))); j++)
		{

			n = read_left_enc();
			diff_l = (vel_l>=0)?(n - left):(left - n); // Left diff will negative for left vel negative
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			n = read_right_enc();
			diff_r = (vel_r>=0)?(n - right):(right - n); // Right diff will negative for right vel negative
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

			angle_diff = (diff_r + diff_l)*dist_per_enc/axle_length;
			if(angle_diff_est>=0)
				angle_diff_final +=angle_diff;
			else
				angle_diff_final -=angle_diff;

//		feedback_.cur_loc
				if(j>=3)
			{
/*  			ddiff += diff_l - diff_r;
  			int vel_change = ddiff/100;
				vel_change = (vel_change>0)?vel_change:-vel_change;
  			if(ddiff>50)
  			{
  				vel_r = vel + vel_change;
  				vel_l = -vel;
  			}
  			else if(ddiff<-50)
  			{
  				vel_l = -vel - vel_change;
  				vel_r = vel;
  			}
  			else
  			{*/
 		if(angle_diff_est >= 0)
		{
			vel_r = 25;
			vel_l = -vel_r;
		}
		else
		{
			vel_l = 25;
			vel_r = -vel_l;
		}  			/*}
  			cout << "Diff L:" << diff_l << " R:" << diff_r << " Diff diff:" << ddiff << endl;*/
			}
			else
			{
//				ddiff = 0;
 		if(angle_diff_est >= 0)
		{
			vel_r = 25;
			vel_l = -vel_r;
		}
		else
		{
			vel_l = 25;
			vel_r = -vel_l;
		}			}

			move_turn(vel_r,vel_l);
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << endl;
			force_angle_range(angle_diff_final);

			r.sleep();
		}

		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		if(j!=0)
			cout << "Left: " << sum_l/j << " Right: " << sum_r/j << endl;

	}

	uint16_t read_left_enc()
	{
		sense[0] = 142;
		sense[1] = 43;
		serial_port_.write(sense, sizeof(sense));
		rw.sleep();
		serial_port_.read(sense, sizeof(sense));
		uint16_t n =  sense[0] | sense[1] << 8;
		return n;
	}

	uint16_t read_right_enc()
	{
		sense[0] = 142;
		sense[1] = 44;
		serial_port_.write(sense, sizeof(sense));
		rw.sleep();
		serial_port_.read(sense, sizeof(sense));
		uint16_t n =  sense[0] | sense[1] << 8;
		return n;
	}

	void force_angle_range(double &angle)
	{
		if(angle > (PI) )
				angle -= 2*PI;
		else
		if(angle < -(PI) )
				angle += 2*PI;
	}
/*    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);

    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }*/

  void executeCB_mod(const rob_nav::navigationGoalConstPtr &goal)
  {
    // helper variables

    bool success = true;
    double x_diff = 0,y_diff = 0;
		double angle_diff_final = 0;

		start();
		safe();

		double x_del = goal->dest.point.x;
		double y_del = goal->dest.point.y;
		if(goal->type==3)
		{
			x_del -= stationary_loc.pose.position.x;
			y_del -= stationary_loc.pose.position.y;
		}



//		cout << "x_del:" << x_del << " y_del:" << y_del << "Init angle:" << stationary_loc.pose.orientation.z << endl;

		double dist_est = 10*pow(pow(x_del,2) +	pow(y_del,2),0.5), dist_final = 0;
//		double angle_diff_est = (x_del==0)?(y_del>=0?PI/2:-PI/2):atan2(y_del,x_del) - stationary_loc.pose.orientation.z;
		double angle_diff_est = (x_del==0)?(y_del>=0?PI/2:-PI/2):atan2(y_del,x_del);
	
//		cout << "Init calc D:" << dist_est << " A:" << angle_diff_est << endl;
		force_angle_range(angle_diff_est);
//		cout << "Init calc D:" << dist_est << " A:" << angle_diff_est << endl;


		// Set turn_dir from goal->turn_dir
		if(goal->type == 0)
		{
			angle_diff_est = (PI/180)*goal->angle;
			force_angle_range(angle_diff_est);
			move_controlled_turn(angle_diff_est,angle_diff_final);
		}
		else	
		if(goal->type == 1 || goal->type == 3)
		{
			move_controlled_turn(angle_diff_est,angle_diff_final);
			rw.sleep();
			move_controlled_straight(dist_est, dist_final);
		}
		else
		if(goal->type == 2)
		{
			int freq = goal->cmd_freq;
			switch(goal->dir)
			{
				case 0: move_timed_straight(1,freq);
								break;
				case 1: move_timed_straight(-1,freq);
								break;
				case 2: move_timed_turn(1,freq);
								break;
				case 3: move_timed_turn(-1,freq);
								break;
				default: break;
			}
		}

		move_straight(0,0);
		rw.sleep();

		//Correct the expected goal position using calculated differences
		stationary_loc.pose.position.x += dist_final*cos(angle_diff_final)/10;
		stationary_loc.pose.position.x += dist_final*sin(angle_diff_final)/10;
		stationary_loc.pose.orientation.z += (180/PI)*angle_diff_final;
		force_angle_range(stationary_loc.pose.orientation.z);

		stationary_loc.header = goal->dest.header;
  	stationary_loc.header.stamp = ros::Time::now();

		result_.final_loc.point = stationary_loc.pose.position;
		result_.angle = stationary_loc.pose.orientation.z;
		as_.setSucceeded(result_);

  }

  void executeCB_square(const rob_nav::navigationGoalConstPtr &goal)
  {
    // helper variables

		start();
		safe();

		int linear = 100, angular = 162.5;

		for(int y=1;y<=1;y++)
		{
			for(int z=1;z<=4; z++)
			{

	/*		move_controlled_straight(); // Put next coordinates
				rw.sleep();
				move_controlled_turn(); // Turn left
	*/
			}

			for(int z=1;z<=4; z++)
			{
	/*		move_controlled_straight(); // Put next coordinates
				rw.sleep();
				move_controlled_turn(); // Turn right
	*/
			}
		}
		result_.final_loc = goal->dest;
		as_.setSucceeded(result_);

  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_server");

  navigationAction nav("nav_act");
  ros::spin();

  return 0;
}
