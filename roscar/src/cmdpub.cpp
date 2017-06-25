#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc,char** argv)
{
		ros::init(argc,argv,"cmd_pub");
		ros::NodeHandle n;
		ros::Publisher cmdVelPub;
		cmdVelPub=n.advertise<geometry_msgs::Twist>("cmd_vel",10);
		double linear_speed = 0.0;
		double linear_speed_max=2.0;
		double a=0.5;
	    double goal_distance = 20.0;
	    double linear_duration = 32.0;
	 
	    double angular_speed = 1.0;
	    double goal_angle = 3.14;
	    double angular_duration = goal_angle / angular_speed;
	 
	    geometry_msgs::Twist speed;
		ros::Rate loopRate(20);
		int rate=20;
	    	while(ros::ok())
	    	//for(int i=0; i<1; i++)
	    	{
			speed = geometry_msgs::Twist();   
		 
			int ticks = int(linear_duration * rate);
			//speed.linear.x = linear_speed;
			//speed.angular.z = angular_speed;
		 
			int t = 0;
			for(t=0; t<ticks/2; t++)
			{
				if(linear_speed<1.0)
				{	linear_speed=linear_speed+a*1.0/25;speed.linear.x = linear_speed;
				}
		    		cmdVelPub.publish(speed);
		    		loopRate.sleep();
		    		//sleep(1);
			}
			for(t=0; t<ticks/2; t++)
			{
				if(linear_speed>0.0)
				{	linear_speed=linear_speed-a*1.0/25;speed.linear.x = linear_speed;
				}
		    		cmdVelPub.publish(speed);
		    		loopRate.sleep();
		    		//sleep(1);
			}
			//在旋转前停止机器人行走
			speed = geometry_msgs::Twist();
			cmdVelPub.publish(geometry_msgs::Twist());
			sleep(1);
			//loopRate.sleep();
	 
			speed.angular.z = angular_speed;
			ticks = int(goal_angle * rate);
			for(t=0; t<ticks; t++)
			{
		    		cmdVelPub.publish(speed);
		    		loopRate.sleep();
			}
	 
			cmdVelPub.publish(geometry_msgs::Twist());
			loopRate.sleep();
	   	 }
		ros::spin();
		return 0;
}
