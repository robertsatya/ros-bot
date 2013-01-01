#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <rob_nav/navigationAction.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <serial/serial.h>
#include <std_msgs/String.h>

#define PI 3.141592

using namespace std;

class navigationAction
{
protected:

  ros::NodeHandle nh_,n1,n2,n3,n4;
  actionlib::SimpleActionServer<rob_nav::navigationAction> as_;
  std::string action_name_;
  rob_nav::navigationFeedback feedback_;
  rob_nav::navigationResult result_;
  geometry_msgs::PoseStamped stationary_loc;
	geometry_msgs::Point target;
	int fd;
	serial::Serial serial_port_;
	ros::Rate rw,r;
	uint8_t byte[5],sense[2],cmd[1];
	double dist_per_enc;
	double dist_per_enc_ang;
	double dist_per_enc_ang_min;
	double dist_per_enc_ang_max;
	double dist_per_enc_ang_90;

	double dist_scale;
	double axle_length;
	int vel;
	int turn_dir; // 0-No turn, 1-Left, 2-Right
	int time_burst_sleep;
	int success;
	string ultra_sts1, ultra_sts2, ultra_sts3, ultra_sts4;
	ros::Subscriber ultra_sub1, ultra_sub2, ultra_sub3, ultra_sub4;
	int mode;
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
		success = 1;
    	as_.start();
		dist_per_enc = 0.003451*56.5/100; //72*PI/65536;
		dist_per_enc_ang_min = 0.003451*75.5/100;
		dist_per_enc_ang_90 = 0.003451*58.5/100;
		dist_per_enc_ang_max = 0.003451*50.5/100;
		dist_per_enc_ang = 0.003451*73.5/100;
		dist_scale = 1;
		reset();
		sleep(5);
		start();
		r.sleep();
		safe();
		r.sleep();
		serial_port_.flushInput();
		r.sleep();
		serial_port_.flushOutput();
		r.sleep();
		ultra_sts1 = "0";
		ultra_sts2 = "0";
		ultra_sts3 = "0";
		ultra_sts4 = "0";
		ultra_sub1 = n1.subscribe("ultra1", 1, &navigationAction::ultraCB1, this);
		ultra_sub2 = n2.subscribe("ultra2", 1, &navigationAction::ultraCB2, this);
		ultra_sub3 = n3.subscribe("ultra3", 1, &navigationAction::ultraCB3, this);
		ultra_sub4 = n4.subscribe("ultra4", 1, &navigationAction::ultraCB4, this);
		mode = 0;


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

	void reset()
	{
		cmd[0] = 7;
		serial_port_.write(cmd, sizeof(cmd));
		rw.sleep();
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

	void ultraCB1(const std_msgs::String::ConstPtr& sts)
	{
		ultra_sts1 = sts->data;
//		cout << "Received u1:" << ultra_sts1 << endl;
	}

	void ultraCB2(const std_msgs::String::ConstPtr& sts)
	{
		ultra_sts2 = sts->data;
//  	cout << "Received u2:" << ultra_sts2 << endl;
	}

	void ultraCB3(const std_msgs::String::ConstPtr& sts)
	{
		ultra_sts3 = sts->data;
	//	cout << "Received u3: " << ultra_sts3 << endl;
	}

	void ultraCB4(const std_msgs::String::ConstPtr& sts)
	{
		ultra_sts4 = sts->data;
//		cout << "Received u4: " << ultra_sts4 << endl;
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
 		byte[0] = 145;
 		byte[1] = right_vel >> 8;
 		byte[2] = right_vel & 0x00ff;
 		byte[3] = left_vel >> 8;
 		byte[4] = left_vel & 0x00ff;
		serial_port_.write(byte, 5);
		rw.sleep();
	}

	void move_timed_straight(int dir, int freq, double angle_turned, double &x_diff, double &y_diff, double &angle_diff_final)
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

		double angle1= 0 , angle2 = 0, angle_diff_final_init = angle_diff_final, dist_final = 0, y_offset_final = 0;
		double dist_step = 0, cur_diff = 0, angle_diff = 0, angle_diff_dist = 0, theta_diff_term = 0, theta_diff_final = 0;

		for(int i=0; i<freq; i++)
		{
			move_straight(25*dir,25*dir);
			usleep(4*time_burst_sleep);
			move_straight(0,0);
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

			double sign = (diff_r-diff_l>=0)?1:-1;
			double sign1 = (dir>=0)?1:-1;

			double cur_diff = diff_r - diff_l;
			angle_diff = (diff_r - diff_l)*dist_per_enc/axle_length;
			double R = (cur_diff!=0)?(diff_r+diff_l)*axle_length/(2*cur_diff):0;
			R = (R>=0)?R:-R;
			double sign_angle = (angle_diff>=0)?1:-1;
			angle_diff = (angle_diff>=0)?angle_diff:-angle_diff;

			if(sign == -1)
			{
				if(sign1 == -1)
				{
					angle1 = angle_diff_final;
					angle2 = angle1 + angle_diff;
				}
				else
				{
					angle1 = angle_diff_final;
					angle2 = angle1 - angle_diff;
					R = -R;
				}
			}
			else
			{
				if(sign1 == -1)
				{
					angle1 = angle_diff_final;
					angle2 = angle1 - angle_diff;
					R = -R;
				}
				else
				{
					angle1 = angle_diff_final;
					angle2 = angle1 + angle_diff;
				}
			}

			double x_diff_term = R*(sin(angle2)-sin(angle1));
			double y_diff_term = R*(cos(angle1)-cos(angle2));
			double r_dist = dist_scale*pow(pow(x_diff_term,2) + pow(y_diff_term,2),0.5);


				theta_diff_term = (x_diff_term==0)?(y_diff_term>=0?(y_diff_term>0?PI/2:0):-PI/2):atan2(y_diff_term,x_diff_term) - angle_diff_final_init;
					angle_diff_final = angle2;


					theta_diff_final += sign*theta_diff_term;
					dist_final += r_dist*cos(theta_diff_final);
					y_offset_final += r_dist*sin(theta_diff_final);
					x_diff += r_dist*cos(theta_diff_final+angle_turned+angle_diff_final_init);
					y_diff += r_dist*sin(theta_diff_final+angle_turned+angle_diff_final_init);

				force_angle_range(angle_diff_final);
				force_angle_range(theta_diff_final);


			cout << "\nDist final:" << dist_final << " Dist step:" << x_diff_term << "Y offset final" << y_offset_final << "Offset step:" << y_diff_term << " rdist:" << r_dist << endl;
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << " Theta diff final:" << theta_diff_final << " Theta diff term:" << theta_diff_term << endl;


			usleep(4*time_burst_sleep);
			int ultra_l=0, ultra_r=0, ultra_b=0, ultra_t=0; // Left 1 Right 2 Bottom 3 Top 4
			double d=0;
			cout << "Ultra in action " << ultra_sts1 << ultra_sts2 << ultra_sts3 << ultra_sts4  << endl;
	   		ultra_l = boost::lexical_cast<int>(ultra_sts1);
	   		ultra_r = boost::lexical_cast<int>(ultra_sts2);
	   		ultra_b = boost::lexical_cast<int>(ultra_sts3);
			ultra_t = boost::lexical_cast<int>(ultra_sts4);
			double cur_ultra_angle = angle_diff_final;
			if(ultra_l == 1 && mode!=3)
			{
				// Turn right
				move_controlled_turn(-0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);

				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;
			}
			else
			if(ultra_r == 1 && mode!=3)
			{
				// Turn left
				move_controlled_turn(0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);

				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;
			}
			else
			if(ultra_b == 1 && mode!=3)
			{
				// For bucket
				move_controlled_turn(0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);

				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;
			}
			else
			if(ultra_t == 1 && mode!=3)
			{
				// Turn left
				move_controlled_turn(0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);

				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;

			}
		}
		
		
	}

	void move_timed_turn(int dir, int freq, double angle_turned, double &x_diff, double &y_diff, double &angle_diff_final)
	{
		uint16_t n=0,left=0,right=0;
		long diff_l=0,diff_r=0;

		double angle_diff = 0, angle_diff_dist = 0, theta_diff_term = 0, theta_diff_final = 0;
		double angle1= 0 , angle2 = 0, angle_diff_final_init = angle_diff_final, dist_final = 0, y_offset_final = 0;

		int vel_r = 25*dir, vel_l = -25*dir;
		n = read_left_enc();
		cout << "Left: " << n << endl;
		left = n;
		rw.sleep();

		n = read_right_enc();
		cout << "Right: " << n << endl;
		right = n;

		for(int i=0; i<freq; i++)
		{
			move_turn(vel_r,vel_l);
			usleep(4*time_burst_sleep);
			move_straight(0,0);

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

			double sign = (diff_r-diff_l>=0)?1:-1;
			double sign1 = (dir>=0)?1:-1;

			double cur_diff = diff_r + diff_l;
			angle_diff = (diff_r + diff_l)*dist_per_enc_ang/axle_length;
			double R = (cur_diff!=0)?(diff_r-diff_l)*axle_length/(2*cur_diff):0;
			R = (R>=0)?R:-R;

			if(sign == -1)
			{
				if(sign1 == -1)
				{
					angle1 = angle_diff_final;
					angle2 = angle1 + angle_diff;
				}
				else
				{
					angle1 = angle_diff_final;
					angle2 = angle1 - angle_diff;
//					R = -R;
				}
			}
			else
			{
				if(sign1 == -1)
				{
					angle1 = angle_diff_final;
					angle2 = angle1 - angle_diff;
//					R = -R;
				}
				else
				{
					angle1 = angle_diff_final;
					angle2 = angle1 + angle_diff;
				}
			}

			double x_diff_term = R*(sin(angle2)-sin(angle1));
			double y_diff_term = R*(cos(angle1)-cos(angle2));
			double r_dist = dist_scale*pow(pow(x_diff_term,2) + pow(y_diff_term,2),0.5);


				theta_diff_term = (x_diff_term==0)?(y_diff_term>=0?(y_diff_term>0?PI/2:0):-PI/2):atan2(y_diff_term,x_diff_term) - angle_diff_final_init;


					angle_diff_final = angle2;

						theta_diff_final += theta_diff_term;
						dist_final += r_dist*cos(theta_diff_final);
						y_offset_final += r_dist*sin(theta_diff_final);
					x_diff += r_dist*cos(theta_diff_final+angle_turned+angle_diff_final_init);
					y_diff += r_dist*sin(theta_diff_final+angle_turned+angle_diff_final_init);

				force_angle_range(angle_diff_final);
				force_angle_range(theta_diff_final);


				cout << "\nX:" << x_diff << " Dist step:" << x_diff_term << "Y:" << y_diff << "Offset step:" << y_diff_term << " rdist:" << r_dist << endl;
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << " Theta diff final:" << theta_diff_final << " Theta diff term:" << theta_diff_term << endl;


			usleep(4*time_burst_sleep);
		}
	}

	void move_controlled_straight(double dist, double angle_turned, double &dist_final, double &x_diff, double &y_diff, double &angle_diff_final)
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

		int vel_r=25, vel_l=25, vel_cur = 0, vel_step = 10;
		vel_cur = vel_step;
		long ddiff = 0;

		double dist_step = 0, cur_diff = 0, angle_diff = 0, angle_diff_dist = 0, theta_diff_term = 0, theta_diff_final = 0, y_offset_final = 0, dist_crit = 370;
		double angle_diff_final_init = angle_diff_final;
		double dist_err = 250;
		double angle1 = 0, angle2=0;
		cout << "Dist target:" << dist << "Dist final" << dist_final << "Angle turned" << angle_turned << "X diff" << x_diff << "Y diff" << y_diff << "Angle diff final" << angle_diff_final << endl;

		int j = 0;
		//Coordinate in mm
		for(j=0; dist_final<=dist-dist_err; j++)
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
				if(vel_cur < vel && dist_final < dist/2)
				{
					vel_cur += vel_step;
				}
				else
				if( dist - dist_final <= dist_crit && vel_cur > 0)
				{
					vel_cur -= vel_step;
				}



  			ddiff += diff_l - diff_r;
  			int vel_change = ddiff/100;
				vel_change = (vel_change>0)?vel_change:-vel_change;

  			if(ddiff>50)
  			{
  				vel_r = vel_cur + vel_change;
  				vel_l = vel_cur;
  			}
  			else if(ddiff<-50)
  			{
  				vel_l = vel_cur + vel_change;
  				vel_r = vel_cur;
  			}
  			else
  			{
  				vel_l = vel_cur;
					vel_r = vel_cur;
  			}
  			cout << "Diff L:" << diff_l << " R:" << diff_r << " Diff diff:" << ddiff << endl;
			}
			else
			{
				ddiff = 0;
				vel_r = vel_step;
				vel_l = vel_step;
			}


			double sign = (diff_r-diff_l>=0)?1:-1;
			double sign1 = 1;

			double cur_diff = diff_r - diff_l;
			angle_diff = (diff_r - diff_l)*dist_per_enc/axle_length;
			double R = (cur_diff!=0)?(diff_r+diff_l)*axle_length/(2*cur_diff):0;
			R = (R>=0)?R:-R;
			double sign_angle = (angle_diff>=0)?1:-1;
			angle_diff = (angle_diff>=0)?angle_diff:-angle_diff;

			if(sign == -1)
			{
				if(sign1 == -1)
				{
					angle1 = angle_diff_final;
					angle2 = angle1 + angle_diff;
				}
				else
				{
					angle1 = angle_diff_final;
					angle2 = angle1 - angle_diff;
					R = -R;
				}
			}
			else
			{
				if(sign1 == -1)
				{
					angle1 = angle_diff_final;
					angle2 = angle1 - angle_diff;
					R = -R;
				}
				else
				{
					angle1 = angle_diff_final;
					angle2 = angle1 + angle_diff;
				}
			}

			double x_diff_term = R*(sin(angle2)-sin(angle1));
			double y_diff_term = R*(cos(angle1)-cos(angle2));
			double r_dist = dist_scale*pow(pow(x_diff_term,2) + pow(y_diff_term,2),0.5);


			if(j>2)
			{
				theta_diff_term = (x_diff_term==0)?(y_diff_term>=0?(y_diff_term>0?PI/2:0):-PI/2):atan2(y_diff_term,x_diff_term) - angle_diff_final_init;
					angle_diff_final = angle2;


					theta_diff_final += sign*theta_diff_term;
					dist_final += r_dist*cos(theta_diff_final);
					y_offset_final += r_dist*sin(theta_diff_final);
					x_diff += r_dist*cos(theta_diff_final+angle_turned+angle_diff_final_init);
					y_diff += r_dist*sin(theta_diff_final+angle_turned+angle_diff_final_init);


				force_angle_range(angle_diff_final);
				force_angle_range(theta_diff_final);

			}

			cout << "\nDist final:" << dist_final << " Dist step:" << x_diff << "Y offset final" << y_offset_final << "Offset step:" << y_diff << " rdist:" << r_dist << endl;
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << " Theta diff final:" << theta_diff_final << " Theta diff term:" << theta_diff_term << "angle init" << angle_turned << endl;
			int ultra_l=0, ultra_r=0, ultra_b=0, ultra_t=0; // Left 1 Right 2 Bottom 3 Top 4
			double d=0;
			cout << "Ultra in action " << ultra_sts1 << ultra_sts2 << ultra_sts3 << ultra_sts4  << endl;
	   		ultra_l = boost::lexical_cast<int>(ultra_sts1);
	   		ultra_r = boost::lexical_cast<int>(ultra_sts2);
	   		ultra_b = boost::lexical_cast<int>(ultra_sts3);
			ultra_t = boost::lexical_cast<int>(ultra_sts4);
			double cur_ultra_angle = angle_diff_final;
			if(ultra_l == 1)
			{
				// Turn right
				move_controlled_turn(-0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);
				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;
			}
			else
			if(ultra_r == 1)
			{
				// Turn left
				move_controlled_turn(0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);
				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;
			}
			else
			if(ultra_b == 1 && mode!=3)
			{
				// For bucket
				move_controlled_turn(0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);
				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;
			}
			else
			if(ultra_t == 1)
			{
				// Turn left
				move_controlled_turn(0.78,cur_ultra_angle,angle_diff_final,x_diff,y_diff);
				move_controlled_straight(500, cur_ultra_angle, d, x_diff, y_diff, angle_diff_final);
				success = 0;
				move_straight(0,0);
				return;
			}

			move_straight(vel_r,vel_l);
			
			r.sleep();
			if(vel_cur == 0)
			{
				break;
			}
		}


		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		if(j!=0)
			cout << "Left: " << sum_l/j << " Right: " << sum_r/j << endl;

	}

	void move_controlled_turn(double angle_diff_est, double angle_turned, double &angle_diff_final, double &x_diff, double &y_diff)
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

		double angle_diff = 0, angle_diff_dist = 0, theta_diff_term = 0, theta_diff_final = 0;
		double angle1 = 0, angle2 = 0;
		double angle_diff_final_init = angle_diff_final;


		double y_offset_final = 0, dist_final = 0;
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
		double lim_del = 0.05,vel_turn = 25;
		
		for(; !((angle_diff_final<(angle_diff_est+lim_del)) && ((angle_diff_final) > (angle_diff_est-lim_del))); j++)
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

			double sign = (diff_r-diff_l>=0)?1:-1;
			double sign1 = (angle_diff_est>=0)?1:-1;

			double cur_diff = diff_r + diff_l;
			double angle_diff_est_calc = 0;
			angle_diff_est_calc = dist_per_enc_ang_max;

			angle_diff = (diff_r + diff_l)*(angle_diff_est_calc)/axle_length;
			double R = (cur_diff!=0)?(diff_r-diff_l)*axle_length/(2*cur_diff):0;
			R = (R>=0)?R:-R;

			if(sign1 == -1)
			{
				angle1 = angle_diff_final;
				angle2 = angle1 - angle_diff;
//				R = -R;
			}
			else
			{
				angle1 = angle_diff_final;
				angle2 = angle1 + angle_diff;
			}

			double x_diff_term = R*(sin(angle2)-sin(angle1));
			double y_diff_term = R*(cos(angle1)-cos(angle2));
			double r_dist = dist_scale*pow(pow(x_diff_term,2) + pow(y_diff_term,2),0.5);


			if(j>2)
			{
				theta_diff_term = (x_diff_term==0)?(y_diff_term>=0?(y_diff_term>0?PI/2:0):-PI/2):atan2(y_diff_term,x_diff_term) - angle_diff_final_init;

					angle_diff_final = angle2;

						theta_diff_final += theta_diff_term;
						dist_final += r_dist*cos(theta_diff_final);
						y_offset_final += r_dist*sin(theta_diff_final);
					x_diff += r_dist*cos(theta_diff_final+angle_turned+angle_diff_final_init);
					y_diff += r_dist*sin(theta_diff_final+angle_turned+angle_diff_final_init);


				force_angle_range(angle_diff_final);
				force_angle_range(theta_diff_final);

			}

			vel_r = sign1*vel_turn;
			vel_l = -vel_r;
		  	
			cout << "\nX:" << x_diff << " Dist step:" << x_diff_term << "Y:" << y_diff << "Offset step:" << y_diff_term << " rdist:" << r_dist << endl;
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << " Theta diff final:" << theta_diff_final << " Theta diff term:" << theta_diff_term << endl;

			
			move_turn(vel_r,vel_l);
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

  void executeCB_mod(const rob_nav::navigationGoalConstPtr &goal)
  {
    // helper variables

    bool success = true;
    double x_diff = 0,y_diff = 0;
		double angle_diff_final = 0;

		start();
		r.sleep();
		safe();
		r.sleep();
		serial_port_.flushInput();
		r.sleep();
		serial_port_.flushOutput();
		r.sleep();


		success = 1;
		target = goal->dest.point;
		mode = goal->type;
		double x_del = goal->dest.point.x;
		double y_del = goal->dest.point.y;
		double angle_diff_est = (x_del==0)?(y_del>=0?(y_del>0?PI/2:0):-PI/2):atan2(y_del,x_del);
		double dist_est = pow(pow(x_del,2) +	pow(y_del,2),0.5), dist_final = 0;

		x_del = dist_est*cos(angle_diff_est);
		y_del = dist_est*sin(angle_diff_est);
		if(goal->type==3)
		{
			x_del -= stationary_loc.pose.position.x;
			y_del -= stationary_loc.pose.position.y;
		}
		angle_diff_est = (x_del==0)?(y_del>=0?(y_del>0?PI/2:0):-PI/2):atan2(y_del,x_del);
		dist_est = pow(pow(x_del,2) +	pow(y_del,2),0.5);

		double cur_angle_rad = (PI/180)*stationary_loc.pose.orientation.z;
		double angle_diff_est_rel = angle_diff_est - cur_angle_rad;


		force_angle_range(angle_diff_est_rel);
		cout << "Init calc D:" << dist_est << " A:" << angle_diff_est <<  " A-rel:" << angle_diff_est_rel <<endl;

			int freq = 1;

		// Set turn_dir from goal->turn_dir
		if(goal->type == 0)
		{
			angle_diff_est_rel = (PI/180)*goal->angle;
			force_angle_range(angle_diff_est_rel);
			move_controlled_turn(angle_diff_est_rel,cur_angle_rad,angle_diff_final,x_diff,y_diff);
		}
		else
		if(goal->type == 1 || goal->type == 3)
		{
			move_controlled_turn(goal->type==3?angle_diff_est_rel:angle_diff_est,cur_angle_rad,angle_diff_final,x_diff,y_diff);
//				move_controlled_turn(angle_diff_est_rel,cur_angle_rad,angle_diff_final,x_diff,y_diff);

			x_del -= x_diff;
			y_del -= y_diff;
			dist_est = pow(pow(x_del,2) +	pow(y_del,2),0.5);
			rw.sleep();
			move_controlled_straight(10*dist_est, cur_angle_rad, dist_final, x_diff, y_diff, angle_diff_final);
		}
		else
		if(goal->type == 2)
		{
			freq = goal->cmd_freq;
			switch(goal->dir)
			{
				case 0: move_timed_straight(1,freq,cur_angle_rad,x_diff,y_diff,angle_diff_final);
								break;
				case 1: move_timed_straight(-1,freq,cur_angle_rad,x_diff,y_diff,angle_diff_final);
								break;
				case 2: move_timed_turn(1,freq,cur_angle_rad,x_diff,y_diff,angle_diff_final);
								break;
				case 3: move_timed_turn(-1,freq,cur_angle_rad,x_diff,y_diff,angle_diff_final);
								break;
				default: break;
			}
		}

		move_straight(0,0);
		rw.sleep();

		//Correct the expected goal position using calculated differences
		stationary_loc.pose.position.x += x_diff/10;
		stationary_loc.pose.position.y += y_diff/10;
		cout << "init angle" << stationary_loc.pose.orientation.z << "final angle diff:" << angle_diff_final << endl;

		double temp = angle_diff_final + (PI/180)*stationary_loc.pose.orientation.z;
		force_angle_range(temp);
		stationary_loc.pose.orientation.z = (180/PI)*temp;

		if(goal->type==4)
		{
			stationary_loc.pose.position.x = 0;
			stationary_loc.pose.position.y = 0;
			stationary_loc.pose.position.z = 0;
			stationary_loc.pose.orientation.z = 0;
			cout << "Odometry reset";
		}
		else if(goal->type==5)
		{
			reset();
		}

		stationary_loc.header = goal->dest.header;
  	stationary_loc.header.stamp = ros::Time::now();

		result_.final_loc.point = stationary_loc.pose.position;
		result_.angle = stationary_loc.pose.orientation.z;
		result_.success = success;
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
