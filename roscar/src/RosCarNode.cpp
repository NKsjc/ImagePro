#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"  //for tf::getPrefixParam
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/server.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "common_def.h"
#include "car_velo.h"
#include "rs232.h"
#include <std_msgs/String.h>
//线程操作需要
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

float v=0.0,w=0.0;//初始化数据，速度与角速度
int fd=-2;//用于判断是否创建链接的句柄
FILE *fp;//用于保存每次运行的里程计信息
int RATE=100;//与底层通信的频率
int i=0,o=0,l=0,r=0;
//float x=0.0,y=0.0,theta=0.0;


void *fun1(void *args)
{
	char nIndex;char *p_nOIndex; float *p_fX;float *p_fY; float *p_fARad;//临时变量
	char p_l1, p_l2,p_r1, p_r2;//临时变量，测试使用
	fp=fopen("coder_new.txt","w+");//以追加的方式写入文件
	ros::Rate looprate(RATE);
	//for(int ii=0;ii<=4;ii++)
	//{
	
	while(ros::ok())
	{
		i=car_get_sended();
	    o=car_get_recved();
		ROS_INFO( "new speed: [%0.2f,%0.2f]", v, w );
		ROS_INFO( "send %d received %d",i,o);
		car_vw_outin(fd,fp,nIndex,v,w,p_nOIndex,p_fX,p_fY,p_fARad, p_l1, p_l2,p_r1, p_r2,l,r);
		ROS_INFO("Coder %d %d", l,r);
		//fprintf(fp,"%d  %d  \n", l,r);
		looprate.sleep();
		//sleep(10);
	}
}
void *fun2(void *args)
{
	//定义需要的变量类型，为待发布的类型
	ros::Publisher pose_pub;
	ros::NodeHandle nh;
	pose_pub=nh.advertise<geometry_msgs::Pose2D>("pose",1);
	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::TransformStamped odom_trans; 
	geometry_msgs::Pose2D pose_2d;
	ros::Rate loop(20);
	while(ros::ok())
	{
		pose_2d.x=car_get_x();
		pose_2d.y=car_get_y();
		pose_2d.theta=car_get_theta();
		pose_pub.publish(pose_2d);
			//looprate.sleep();
			//pos = robot->getPose();
		   // publishing transform odom->base_link
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.header.frame_id = frame_id_odom;
		odom_trans.child_frame_id = frame_id_base_link;
		  
		odom_trans.transform.translation.x = pose_2d.x;
		odom_trans.transform.translation.y = pose_2d.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(pose_2d.theta);  
		odom_broadcaster.sendTransform(odom_trans);
		loop.sleep();
	}
}
void cmdCallback(const geometry_msgs::TwistConstPtr &msg)
{
	v=msg->linear.x;
	w=msg->angular.z;
	ROS_INFO( "shoudao  new speed: [%0.2f,%0.2f]", v, w );
}
int main(int argc,char **argv)
{
	ros::init(argc,argv,"ros_car_node");
	ros::NodeHandle n;
	ros::Subscriber wheelsub;
	// cmdvel_sub = n.subscribe( "cmd_vel", 1, (boost::function <void(const geometry_msgs::TwistConstPtr&)>)
    //  boost::bind(&RosAriaNode::cmdvel_cb, this, _1 ));
	//wheelsub = n.subscribe("cmd_vel",10,(boost::function <void(const geometry_msgs::TwistConstPtr&)>)boost::bind(&cmdCallback,_1 ));
	wheelsub=n.subscribe<geometry_msgs::Twist>("cmd_vel",10,&cmdCallback);
	//ros::spin();
	
	pthread_t pid1,pid2;

	fd=car_velo_init(PORT);//初始化串口节点
	//判断是否建立成功
	if(fd<0)
	{
		ROS_INFO("串口初始化失败");
		//
	}
	else
		ROS_INFO("sucessed");
	if(pthread_create(&pid1,NULL,fun1,NULL))
	{
		return -1;
	}
	if(pthread_create(&pid2,NULL,fun2,NULL))
	{
		return -1;
	}
	pthread_join(pid1,NULL);
	pthread_join(pid2,NULL);
	ros::MultiThreadedSpinner spinner(3); // Use 2 threads
	 spinner.spin(); // spin() will not return until the node has been shutdown 
	
}
