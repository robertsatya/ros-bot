#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
using namespace std;

#define PI 3.141592

	void force_angle_range(double &angle)
	{
		if(angle > (PI) )
				angle -= 2*PI;
		else
		if(angle < -(PI) )
				angle += 2*PI;
	}

	void move_straight(unsigned int right_vel, unsigned int left_vel)
	{
 		cout << "Moving forward at L:" << left_vel << " R:" << right_vel << " mm/s\n";
/* 		byte[0] = 145;
 		byte[1] = right_vel >> 8;
 		byte[2] = right_vel & 0x00ff;
 		byte[3] = left_vel >> 8;
 		byte[4] = left_vel & 0x00ff;
		serial_port_.write(byte, 5);*/
		usleep(20000);
	}

int main(int argc, char* argv[])
{
	double dist_per_enc = 0.003451*56.5/100; //72*PI/65536;
	double dist_per_enc_ang = 0.003451*51/100;
	double vel = 200, axle_length = 235;

	string fn = argv[7];
	ifstream ifs(fn.c_str());
	unsigned int lenc,renc;
	
		int step_br = atoi(argv[1]);
		double dist = atof(argv[2]), angle_turned = atof(argv[3]), x_diff = atof(argv[4]), y_diff = atof(argv[5]), angle_diff_final = atof(argv[6]), dist_final = 0;
		long sum_l=0,sum_r=0;
		unsigned int n=0,left=0,right=0;
		long diff_l=0,diff_r=0;

		n = 0;
		cout << "Left: " << n << endl; 
		left = n;
		usleep(20000);

		n = 0;
		cout << "Right: " << n << endl; 
		right = n;

		int vel_r=25, vel_l=25, vel_cur = 0, vel_step = 10;
		vel_cur = vel_step;
		long ddiff = 0;

		double dist_step = 0, cur_diff = 0, angle_diff = 0, angle_diff_dist = 0, theta_diff_term = 0, theta_diff_final = 0, y_offset_final = 0, dist_crit = 370;
		double dist_err = dist/15;
		double angle1 = 0, angle2=0;
//		double angle_diff_final = 0;
		cout << "Dist target:" << dist << "Dist final" << dist_final << "Angle turned" << angle_turned << "X diff" << x_diff << "Y diff" << y_diff << "Angle diff final" << angle_diff_final << endl;

	//	return;
		//dist_final = 0;
		int j = 0;
		//Coordinate in mm
		for(j=0; dist_final<(dist) && !ifs.eof() && j<step_br; j++)
		{

			ifs>>lenc>>renc;
  		n = lenc;
			diff_l = n - left;
			diff_l = (diff_l<0)?(diff_l+65535):(diff_l);
			cout << "Left: " << n << "Prev: " << left << " Diff: " << diff_l << endl; 
			left = n;

			n = renc;
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
//  			ddiff += diff_l - diff_r;
				ddiff = 0;
				vel_r = vel_step;
				vel_l = vel_step;
			}
	
//		feedback_.cur_loc

			move_straight(vel_r,vel_l);

/* MODIFIED CODE */ 

			double sign = (diff_r-diff_l>=0)?1:-1;
			double sign1 = 1;

			double cur_diff = diff_r - diff_l;
			angle_diff = (diff_r - diff_l)*dist_per_enc_ang/axle_length;
			angle_diff_dist = (cur_diff)*dist_per_enc/axle_length;
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
			double r_dist = pow(pow(x_diff_term,2) + pow(y_diff_term,2),0.5);

				theta_diff_term = (x_diff_term==0)?(y_diff_term>=0?(y_diff_term>0?PI/2:0):-PI/2):atan2(y_diff_term,x_diff_term) - angle_turned;

			if(j>2)
			{
/*				double temp = theta_diff_term;
				if(sign == -1)
				{
					if(sign1 == -1)
					{
						theta_diff_term = PI + theta_diff_term;
					}
					else
					{
						theta_diff_term = -theta_diff_term;
					}
				}
				else
				{
					if(sign1 == -1)
					{
						theta_diff_term= -(-PI + theta_diff_term);
					}
					else
					{
						theta_diff_term = theta_diff_term;
					}
				}
*/
					x_diff += R*(sin(angle2 + angle_turned)-sin(angle1 + angle_turned));
					y_diff += R*(cos(angle1 + angle_turned)-cos(angle2 + angle_turned));


					theta_diff_final += theta_diff_term;
					dist_final += r_dist*cos(theta_diff_final);
					y_offset_final += r_dist*sin(theta_diff_final);

				angle_diff_final += sign*angle_diff;
				force_angle_range(angle_diff_final);
				force_angle_range(theta_diff_final);

			}

			cout << "X_diff:" << x_diff << " Y_diff:" << y_diff << endl;
			cout << "Dist final:" << dist_final << " Dist step:" << x_diff_term << " Y offset final:" << y_offset_final << " Offset step:" << y_diff_term << " rdist:" << r_dist << endl;
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << " Theta diff final:" << theta_diff_final << " Theta diff term:" << theta_diff_term << endl << endl;
			
			
/* ORIGINAL CODE  
			dist_step = get_distance(diff_r,diff_l);
			angle_diff = (diff_r - diff_l)*dist_per_enc/axle_length;

			angle_diff_final += angle_diff;
			dist_final += dist_step;

			cout << "\nDist final:" << dist_final << " Dist step:" << dist_step << endl;
			cout << "Angle diff total:" << angle_diff_final << " Angle diff step:" << angle_diff << endl;
*/
			usleep(125000);
			if(vel_cur == 0)
			{
				break;	
			}
		}

			
		cout << "Total Left: " << sum_l << "Total Right: " << sum_r << endl;
		if(j!=0)
			cout << "Left: " << sum_l/j << " Right: " << sum_r/j << endl;

		
	return 0;
}
