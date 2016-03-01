#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <rob_nav/navigationAction.h>
#include <geometry_msgs/PointStamped.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <serial/serial.h>

using namespace std;

int tcdrain(int fildes);

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
  geometry_msgs::PointStamped stationary_loc;
	int fd;
	serial::Serial serial_port_;


public:

  navigationAction(std::string name) :
    as_(nh_, name, boost::bind(&navigationAction::executeCB_mod, this, _1), false),
    action_name_(name),
		serial_port_("/dev/ttyUSB0",115200,serial::Timeout::simpleTimeout(1000))
  {
		stationary_loc.header.seq = 0;
  	stationary_loc.header.stamp = ros::Time::now();
  	stationary_loc.header.frame_id = "/robot";
  	stationary_loc.point.x = 0;
  	stationary_loc.point.y = 0;
  	stationary_loc.point.z = 0;
//		open_port();
//		configure_port();
    as_.start();

  }

  ~navigationAction(void)
  {
		    try{ 
        serial_port_.close();}
    catch(std::exception& e){
        std::cerr<<e.what()<<std::endl;
        }
  }

  void open_port()
  {
/*  	fd = open("/dev/ttyUSB0", O_RDWR);
  
  	if(fd == -1) {
  		printf("open_port: Unable to open /dev/ttyUSB0. \n");
  	} else {
  		fcntl(fd, F_SETFL, 0);
  		printf("port is open.\n");
  	}*/
	try
	{ 
//        serial_port_.setPort("/dev/ttyUSB0");
 //       serial_port_.setBaudrate(115200);
        serial_port_.open();
		 		printf("port is open.\n");

    }
	catch(std::exception& e){
        std::cerr<<"iRobotCreate2 Error opening serial port: "<<e.what()<<std::endl;
    }
  }

  void configure_port()
  {
  	struct termios port_settings;
  
  	cfsetispeed(&port_settings, B115200);
  	cfsetospeed(&port_settings, B115200);
  
  	port_settings.c_cflag &= ~PARENB;		// set no parity
  	port_settings.c_cflag &= ~CSTOPB;		// set stop bits to 1
  	port_settings.c_cflag &= ~CRTSCTS;		// disable hardware flow control
  	port_settings.c_cflag &= ~CSIZE;		// set databits to 8
  	port_settings.c_cflag |= CS8;
  
  	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // disable software flow control
  
  	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
  }

	void move_straight(int right_vel,int left_vel)
	{
		ros::Rate rw(50);

		uint8_t byte[5];
	 		cout << "Moving forward at L:" << left_vel << " R:" << right_vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = right_vel;
  		byte[3] = 0;
  		byte[4] = left_vel;
		serial_port_.write(byte, 5);
		rw.sleep();

	}

	void move_left(int right_vel,int left_vel)
	{
		ros::Rate rw(50);

		uint8_t byte[5];
	 		cout << "Moving forward at L:" << 256-left_vel << " R:" << right_vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = right_vel;
  		byte[3] = 255;
  		byte[4] = left_vel;
		serial_port_.write(byte, 5);
		rw.sleep();

	}

  void executeCB_straight(const rob_nav::navigationGoalConstPtr &goal)
  {
    // helper variables

		int sense_freq = 2;
		float vel = 100;
		float neg_vel = 156;
    ros::Rate r(sense_freq),rw(50);
    bool success = true;
		uint32_t sum_l=0,sum_r=0;
		uint16_t n=0,left=0,right=0,diff_l=0,diff_r=0;
		char sense[2],byte[5],cmd[1];


		tcflush(fd,TCIOFLUSH);
		cmd[0] = 131;
		write(fd, cmd, sizeof(cmd));
		tcdrain(fd);

		sense[0] = 142;
		sense[1] = 43;
		write(fd, sense, sizeof(sense));
		tcdrain(fd);
		rw.sleep();
		read(fd, sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Left: " << n << endl; 
		left = n;

		rw.sleep();
		sense[0] = 142;
		sense[1] = 44;
		write(fd, sense, sizeof(sense));
		tcdrain(fd);
		rw.sleep();
		read(fd, sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Right: " << n << endl; 
		right = n;

  		cout << "Moving forward at " << vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = vel;
  		byte[3] = 0;
  		byte[4] = vel;
		

		write(fd, byte, sizeof(byte));
		tcdrain(fd);
		rw.sleep();
		
		//Coordinate in mm
		float x = goal->dest.point.x;
		float y = goal->dest.point.y;
		float dist = 25.4*pow(pow(stationary_loc.point.x - goal->dest.point.x,2) +	pow(stationary_loc.point.y - goal->dest.point.y,2),0.5);

		float iter_max = dist*sense_freq/(vel+12.5);
		for(int j=0; j<iter_max; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
  		write(fd, sense, sizeof(sense));
  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
  		write(fd, sense, sizeof(sense));
  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

//		feedback_.cur_loc

			r.sleep();
		}
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max << " Right: " << sum_r/iter_max << endl;


		cout << "Stopping\n";
		byte[0] = 145;
		byte[1] = 0;
		byte[2] = 0;
		byte[3] = 0;
		byte[4] = 0;

		write(fd, byte, sizeof(byte));
		tcdrain(fd);

		rw.sleep();
		result_.final_loc = goal->dest;
		stationary_loc.point = goal->dest.point;
		stationary_loc.header = goal->dest.header;
  	stationary_loc.header.stamp = ros::Time::now();

		as_.setSucceeded(result_);

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
    }
*/
  }

  void executeCB_mod(const rob_nav::navigationGoalConstPtr &goal)
  {
    // helper variables

		int sense_freq = 4;
		float vel = 200;
		float neg_vel = 56;
    ros::Rate r(sense_freq),rw(50);
    bool success = true;
		uint32_t sum_l=0,sum_r=0;
		uint16_t n=0,left=0,right=0;
		long diff_l=0,diff_r=0;
		uint8_t sense[2],byte[5],cmd[1];


//		tcflush(fd,TCIOFLUSH);
		cmd[0] = 128;
//		write(fd, cmd, sizeof(cmd));
		serial_port_.write(cmd, sizeof(cmd));

//		tcdrain(fd);
		rw.sleep();

		cmd[0] = 131;
//		write(fd, cmd, sizeof(cmd));
		serial_port_.write(cmd, sizeof(cmd));

//		tcdrain(fd);
		rw.sleep();

		sense[0] = 142;
		sense[1] = 43;
//		write(fd, sense, sizeof(sense));
		serial_port_.write(sense, sizeof(sense));

//		tcdrain(fd);
		rw.sleep();
//		read(fd, sense, sizeof(sense));
		serial_port_.read(sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Left: " << n << endl; 
		left = n;

		rw.sleep();
		sense[0] = 142;
		sense[1] = 44;
//		write(fd, sense, sizeof(sense));
		serial_port_.write(sense, sizeof(sense));

//		tcdrain(fd);
		rw.sleep();
//		read(fd, sense, sizeof(sense));
  	serial_port_.read(sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Right: " << n << endl; 
		right = n;

		int vel_r=vel, vel_l=vel, vel_n=neg_vel, vel_p = vel;
		long ddiff = 0;

		if(goal->type == 1)
		{
/*  		cout << "Moving forward at " << vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = vel_r;
  		byte[3] = 0;
  		byte[4] = vel_l;
		write(fd, byte, sizeof(byte));
//		tcdrain(fd);
		rw.sleep();*/
			vel_r = vel_l = 25;
			move_straight(vel_r,vel_l);
		
		//Coordinate in cm
		float iter_max1 = goal->dest.point.x*sense_freq/vel;
		for(int j=0; j<iter_max1; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
  	//	write(fd, sense, sizeof(sense));
			serial_port_.write(sense, sizeof(sense));

//  		tcdrain(fd);
			rw.sleep();
//  		read(fd, sense, sizeof(sense));
			serial_port_.read(sense, sizeof(sense));

			n = sense[0] | sense[1] << 8;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
 // 		write(fd, sense, sizeof(sense));
			serial_port_.write(sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
 // 		read(fd, sense, sizeof(sense));
			serial_port_.read(sense, sizeof(sense));

			n = sense[0] | sense[1] << 8;
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
  /*			if(ddiff > 2000)
  				vel_change = 10;
  			else if(ddiff < -2000)
  				vel_change = -10;*/
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
  				vel_l = vel_r = vel;
  			}
  			cout << "Diff L:" << diff_l << " R:" << diff_r << " Diff diff:" << ddiff << endl;
			}
			else
			{
				ddiff = 0;
				vel_r = vel_l = 25;
			}
	
//		feedback_.cur_loc
/*  		cout << "Moving left at L:" << vel_l << " R:" << vel_r << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = vel_r;
  		byte[3] = 0;
  		byte[4] = vel_l;
		write(fd, byte, sizeof(byte));*/
			move_straight(vel_r,vel_l);
			r.sleep();
		}


		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max1 << " Right: " << sum_r/iter_max1 << endl;


		}
		else
		if(goal->type == 2)
		{
/*  		cout << "Moving left at " << vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = vel_p;
  		byte[3] = 255;
  		byte[4] = vel_n;
		write(fd, byte, sizeof(byte));*/
			vel_p = 25;
			vel_n = 256-vel_p;
			move_left(vel_p,vel_n);
//		tcdrain(fd);
		rw.sleep();
		
		//Coordinate in cm
		float iter_max2 = goal->dest.point.x*sense_freq/vel;
		for(int j=0; j<iter_max2; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
//  		write(fd, sense, sizeof(sense));
			serial_port_.write(sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
//  		read(fd, sense, sizeof(sense));
			serial_port_.read(sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_l = left - n; // Left vel is negative
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
//  		write(fd, sense, sizeof(sense));
			serial_port_.write(sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
//  		read(fd, sense, sizeof(sense));
			serial_port_.read(sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

//		feedback_.cur_loc
			if(j>=3)
			{
  			ddiff += diff_l - diff_r;
  			int vel_change = ddiff/100;
				vel_change = (vel_change>0)?vel_change:-vel_change;
  /*			if(ddiff > 2000)
  				vel_change = 10;
  			else if(ddiff < -2000)
  				vel_change = -10;*/
  			if(ddiff>50)
  			{
  				vel_p = vel + vel_change;
  				vel_n = neg_vel;
  			}
  			else if(ddiff<-50)
  			{
  				vel_n = neg_vel - vel_change;
  				vel_p = vel;
  			}
  			else
  			{
  				vel_p = vel;
					vel_n = neg_vel;
  			}
  			cout << "Diff L:" << diff_l << " R:" << diff_r << " Diff diff:" << ddiff << endl;
			}
			else
			{
				ddiff = 0;
				vel_p = 25;
				vel_l = 256-vel_p;
			}

/*  		cout << "Moving left at L:" << vel_n << " R:" << vel_p << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = vel_p;
  		byte[3] = 255;
  		byte[4] = vel_n;
		write(fd, byte, sizeof(byte));
*/
			move_left(vel_p,vel_n);
			r.sleep();
		}

		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max2 << " Right: " << sum_r/iter_max2 << endl;


		}


/*		cout << "Stopping\n";
		byte[0] = 145;
		byte[1] = 0;
		byte[2] = 0;
		byte[3] = 0;
		byte[4] = 0;

		write(fd, byte, sizeof(byte));*/
		move_straight(0,0);
//		tcdrain(fd);

		rw.sleep();
		result_.final_loc = goal->dest;
		as_.setSucceeded(result_);

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
    }
*/
  }

  void executeCB(const rob_nav::navigationGoalConstPtr &goal)
  {
    // helper variables

		int sense_freq = 2;
		float vel = 100;
		float neg_vel = 156;
    ros::Rate r(sense_freq),rw(50);
    bool success = true;
		uint32_t sum_l=0,sum_r=0;
		uint16_t n=0,left=0,right=0,diff_l=0,diff_r=0;
		char sense[2],byte[5],cmd[1];


		tcflush(fd,TCIOFLUSH);
		cmd[0] = 131;
		write(fd, cmd, sizeof(cmd));
//		tcdrain(fd);
		rw.sleep();

		sense[0] = 142;
		sense[1] = 43;
		write(fd, sense, sizeof(sense));
//		tcdrain(fd);
		rw.sleep();
		read(fd, sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Left: " << n << endl; 
		left = n;

		rw.sleep();
		sense[0] = 142;
		sense[1] = 44;
		write(fd, sense, sizeof(sense));
//		tcdrain(fd);
		rw.sleep();
		read(fd, sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Right: " << n << endl; 
		right = n;

		if(goal->type == 1)
		{
  		cout << "Moving forward at " << vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = vel;
  		byte[3] = 0;
  		byte[4] = vel;
		}
		else
		if(goal->type == 2)
		{
  		cout << "Moving left at " << vel << " mm/s\n";
  		byte[0] = 145;
  		byte[1] = 0;
  		byte[2] = vel;
  		byte[3] = 255;
  		byte[4] = neg_vel;
		}

		write(fd, byte, sizeof(byte));
//		tcdrain(fd);
		rw.sleep();
		
		//Coordinate in cm
		float iter_max = goal->dest.point.x*sense_freq/vel;
		for(int j=0; j<iter_max; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

//		feedback_.cur_loc

			r.sleep();
		}
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max << " Right: " << sum_r/iter_max << endl;


		cout << "Stopping\n";
		byte[0] = 145;
		byte[1] = 0;
		byte[2] = 0;
		byte[3] = 0;
		byte[4] = 0;

		write(fd, byte, sizeof(byte));
//		tcdrain(fd);

		rw.sleep();
		result_.final_loc = goal->dest;
		as_.setSucceeded(result_);

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
    }
*/
  }

  void executeCB1(const rob_nav::navigationGoalConstPtr &goal)
  {
    // helper variables

		int sense_freq = 8;
		float vel = 100;
		float neg_vel = 156;
    ros::Rate r(sense_freq),rw(50);
    bool success = true;
		uint32_t sum_l=0,sum_r=0;
		uint16_t n=0,left=0,right=0,diff_l=0,diff_r=0;
		char sense[2],byte[5],byte1[5],byte2[5],cmd[1];


		tcflush(fd,TCIOFLUSH);
		cmd[0] = 131;
		write(fd, cmd, sizeof(cmd));
//		tcdrain(fd);
		rw.sleep();

		sense[0] = 142;
		sense[1] = 43;
		write(fd, sense, sizeof(sense));
//		tcdrain(fd);
		rw.sleep();
		read(fd, sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Left: " << n << endl; 
		left = n;

		rw.sleep();
		sense[0] = 142;
		sense[1] = 44;
		write(fd, sense, sizeof(sense));
//		tcdrain(fd);
		rw.sleep();
		read(fd, sense, sizeof(sense));
		n = sense[0] | sense[1] << 8;
		cout << "Right: " << n << endl; 
		right = n;

		int linear = 100, angular = 162.5;

for(int y=1;y<=1;y++)
{
for(int z=1;z<=4; z++)
{
  		cout << "Moving forward at " << vel << " mm/s\n";
  		byte1[0] = 145;
  		byte1[1] = 0;
  		byte1[2] = vel;
  		byte1[3] = 0;
  		byte1[4] = vel;
 		write(fd, byte1, sizeof(byte1));
//		tcdrain(fd);
		rw.sleep();
		
		//Coordinate in cm

		float iter_max1 = linear*sense_freq/vel;
		for(int j=0; j<iter_max1; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

//		feedback_.cur_loc

			r.sleep();
		}
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max1 << " Right: " << sum_r/iter_max1 << endl;


		cout << "Stopping\n";
		byte[0] = 145;
		byte[1] = 0;
		byte[2] = 0;
		byte[3] = 0;
		byte[4] = 0;

		write(fd, byte, sizeof(byte));
//		tcdrain(fd);

		rw.sleep();

 		cout << "Moving left at " << vel << " mm/s\n";
  		byte2[0] = 145;
  		byte2[1] = 0;
  		byte2[2] = vel;
  		byte2[3] = 255;
  		byte2[4] = neg_vel;
		write(fd, byte2, sizeof(byte2));
//		tcdrain(fd);
		rw.sleep();
	


		float iter_max2 = angular*sense_freq/vel;
		for(int j=0; j<iter_max2; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

//		feedback_.cur_loc

			r.sleep();
		}
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max2 << " Right: " << sum_r/iter_max2 << endl;

		cout << "Stopping\n";
		byte[0] = 145;
		byte[1] = 0;
		byte[2] = 0;
		byte[3] = 0;
		byte[4] = 0;

		write(fd, byte, sizeof(byte));
//		tcdrain(fd);

		rw.sleep();

}

for(int z=1;z<=4; z++)
{
  		cout << "Moving forward at " << vel << " mm/s\n";
  		byte1[0] = 145;
  		byte1[1] = 0;
  		byte1[2] = vel;
  		byte1[3] = 0;
  		byte1[4] = vel;
 		write(fd, byte1, sizeof(byte1));
//		tcdrain(fd);
		rw.sleep();
		
		//Coordinate in cm

		float iter_max1 = linear*sense_freq/vel;
		for(int j=0; j<iter_max1; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

//		feedback_.cur_loc

			r.sleep();
		}
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max1 << " Right: " << sum_r/iter_max1 << endl;


		cout << "Stopping\n";
		byte[0] = 145;
		byte[1] = 0;
		byte[2] = 0;
		byte[3] = 0;
		byte[4] = 0;

		write(fd, byte, sizeof(byte));
//		tcdrain(fd);

		rw.sleep();

 		cout << "Moving left at " << vel << " mm/s\n";
  		byte2[0] = 145;
  		byte2[1] = 255;
  		byte2[2] = neg_vel;
  		byte2[3] = 0;
  		byte2[4] = vel;
		write(fd, byte2, sizeof(byte2));
//		tcdrain(fd);
		rw.sleep();
	


		float iter_max2 = angular*sense_freq/vel;
		for(int j=0; j<iter_max2; j++)
		{

  		sense[0] = 142;
  		sense[1] = 43;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			sense[0] = 142;
			sense[1] = 44;
  		write(fd, sense, sizeof(sense));
//  		tcdrain(fd);
			rw.sleep();
  		read(fd, sense, sizeof(sense));
			n = sense[0] | sense[1] << 8;
			cout << "Right: " << n << "Prev: " << right << " Diff: " << diff_r << endl; 
			diff_r = n - right;
			diff_r = (diff_r<0)?(diff_r+65535):(diff_r);
			right = n;
			
			sum_l += diff_l;
			sum_r += diff_r;
			cout << j << endl; 

//		feedback_.cur_loc

			r.sleep();
		}
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		cout << "Left: " << sum_l/iter_max2 << " Right: " << sum_r/iter_max2 << endl;

		cout << "Stopping\n";
		byte[0] = 145;
		byte[1] = 0;
		byte[2] = 0;
		byte[3] = 0;
		byte[4] = 0;

		write(fd, byte, sizeof(byte));
//		tcdrain(fd);

		rw.sleep();

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
