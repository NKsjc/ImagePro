#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"

#include "common_def.h"
#include "car_velo.h" 
#include "rs232.h"//38400
int fd;
char nIndex;
void cmdCallback(const geometry_msgs::TwistConstPtr & msg)
{
	ros::Time veltime = ros::Time::now();
	int i=car_get_sended();
	int o=car_get_recved();
	 char *p_nOIndex; float *p_fX;float *p_fY; float *p_fARad;
	ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );
	ROS_INFO( "send %d recved %d",i,o);
	car_vw_outin(fd,nIndex,msg->linear.x,msg->angular.z,p_nOIndex,p_fX,p_fY,p_fARad);
}
int main(int argc,char ** argv)
{
//ros::init(argc,argv,"roscar");//  /dev/ttyS3
    ros::init(argc,argv, "roscar");
	char* device="/dev/ttyS2";
	fd=car_velo_init(device);
	ROS_INFO("fd %d",fd);
	//int car_vw_outin(int fd, char nIndex, float fVelo_V, float fVelo_W, char *p_nOIndex, float *p_fX, float *p_fY, float *p_fARad)
	ros::NodeHandle n;
	ros::Subscriber wheelsub = n.subscribe<geometry_msgs::Twist>("cmd_vel",1,&cmdCallback);
	//ros::Publisher statussub = n.publish<>
	ros::spin();
	return 0;
}
