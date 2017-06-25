#include "ros/ros.h"
#include "ros/duration.h"
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

class RosCarNode
{
	public:
		RosCarNode(ros::NodeHandle nh);
		virtual ~RosCarNode();
		
	public:
		int Setup();
		//void cmdvel_cb( const geometry_msgs::TwistConstPtr &);
		//void cmd_enable_motors_cb();
		//void cmd_disable_motors_cb();
		void spin();
		void publish();
		void cmdCallback(const geometry_msgs::TwistConstPtr & msg);
		void testCallback(const std_msgs::String::ConstPtr & str);
		void fun1(const ros::TimerEvent&);
		void fun2(const ros::TimerEvent&);
		
	protected:
		int fd;
		pthread_t pid1;
		int i;
		int l,r;
		int o;
		float v;
		float w;
		float x;
		float y;
		float theta;
		FILE *fp;
		//留着的变量
		char nIndex;char *p_nOIndex; float *p_fX;float *p_fY; float *p_fARad;
		ros::NodeHandle n;
		ros::Subscriber wheelsub;
		ros::Subscriber strsub;
		ros::Publisher pose_pub;
		//for odom->base_link transform
		tf::TransformBroadcaster odom_broadcaster;
		geometry_msgs::TransformStamped odom_trans; 
		geometry_msgs::Pose2D pose_2d;
};
//RosCarNode's initialize
//advertise topic to ros
RosCarNode::RosCarNode(ros::NodeHandle nh) : n(nh)
{
	i=0;o=0;
	v=0.0;w=0.0;l=0;r=0;
	x=0.0;y=0.0;theta=0.0;
	fp=fopen("coder.txt","w+");
	pose_pub=n.advertise<geometry_msgs::Pose2D>("pose",1000);//publish topic named pose
	//在构造函数中写param参数服务器
	//n.param("",,);
	
}
RosCarNode::~RosCarNode()
{	
}
void RosCarNode::spin()
{
	 //ros::MultiThreadedSpinner spinner(2); // Use 2 threads
	 //spinner.spin(); // spin() will not return until the node has been shutdown 
	 ros::spin();
}
void RosCarNode::fun2(const ros::TimerEvent&)
{
	//如何开启线程完成这个任务，目前是消息驱动的ROS主题发布
	    x=car_get_x();
	    y=car_get_y();
	    theta=car_get_theta();
		pose_2d.x=x;
		pose_2d.y=y;
		pose_2d.theta=theta;
		pose_pub.publish(pose_2d);
		//looprate.sleep();
		//pos = robot->getPose();
	   // publishing transform odom->base_link
	  odom_trans.header.stamp = ros::Time::now();
	  odom_trans.header.frame_id = frame_id_odom;
	  odom_trans.child_frame_id = frame_id_base_link;
	  
	  odom_trans.transform.translation.x = x;
	  odom_trans.transform.translation.y = y;
	  odom_trans.transform.translation.z = 0.0;
	  odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);  
	  odom_broadcaster.sendTransform(odom_trans);
}
void RosCarNode::cmdCallback(const geometry_msgs::TwistConstPtr & msg)
{
	ros::Time current_time=ros::Time::now();
	//将速度赋值，从而使线程通信的速度改变
	v=msg->linear.x;
	w=msg->angular.z;
	/*char nIndex;char *p_nOIndex; float *p_fX;float *p_fY; float *p_fARad;
	char p_l1, p_l2,p_r1, p_r2;
	
	ros::Rate looprate(100);
	for(int ii=0;ii<=4;ii++)
	{
		i=car_get_sended();
	    o=car_get_recved();
		ROS_INFO( "new speed: [%0.2f,%0.2f]", msg->linear.x*1e3, msg->angular.z );
		ROS_INFO( "send %d received %d",i,o);
		car_vw_outin(fd,fp,nIndex,1.0*msg->linear.x,msg->angular.z,p_nOIndex,p_fX,p_fY,p_fARad, p_l1, p_l2,p_r1, p_r2,l,r);
		ROS_INFO("Coder %d %d", l,r);
		//fprintf(fp,"%d  %d  \n", l,r);
   }
	*/	
}
void RosCarNode::fun1(const ros::TimerEvent&)
{
	char nIndex;char *p_nOIndex; float *p_fX;float *p_fY; float *p_fARad;//临时变量
	char p_l1, p_l2,p_r1, p_r2;//临时变量，测试使用
	//fp=fopen("coder_new.txt","w+");//以追加的方式写入文件
	//ros::Rate looprate(RATE);
	//for(int ii=0;ii<=4;ii++)
	//{
	//while(ros::ok())
	//{
		i=car_get_sended();
	    o=car_get_recved();
		ROS_INFO( "new speed: [%f,%f]", v, w );
		ROS_INFO( "send %d received %d",i,o);
		car_vw_outin(fd,fp,nIndex,v,w,p_nOIndex,p_fX,p_fY,p_fARad, p_l1, p_l2,p_r1, p_r2,l,r);
		ROS_INFO("Coder %d %d", l,r);
		//fprintf(fp,"%d  %d  \n", l,r);
		//looprate.sleep();
	//}
}
/*void * RosCarNode::comunicate(void * args)
{
	e
}*/
int RosCarNode::Setup()
{
	fd=car_velo_init(PORT);
	/*if(fd==-1)
	{
			ROS_INFO("fd %d init failed",fd);
	}
	else
	{*/
		ROS_INFO("fd %d init succeed",fd);
		int flag;
		char nIndex;float * robot_status;float* battery_status;float* cmd_status;float * motor1_status;float * motor2_status;float * led_status;
		ROS_INFO("Get the status of robot");
		flag=car_status_outin(fd,nIndex,robot_status,battery_status,cmd_status,motor1_status,motor2_status,led_status);
		/*if(pthread_create(&pid1, NULL, comunicate, NULL))
		{
			ROS_INFO("通信线程创建失败");
			return -1;
		}*/
		wheelsub = n.subscribe<geometry_msgs::Twist>("cmd_vel",1,&RosCarNode::cmdCallback,this);
		//strsub =n.subscribe<std_msgs::String>("test",10,&RosCarNode::testCallback,this);
	//}
}

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"RosCar_node");
	int pub_hz;
	ros::NodeHandle nh;
	nh.param("pub_hz",pub_hz,40);
//ros::Timer timer = nh.createTimer(ros::Duration(0.1), &Foo::callback, &foo_object);
	RosCarNode *roscar=new RosCarNode(nh);
	ros::Timer timer1=nh.createTimer(ros::Duration(0.01),&RosCarNode::fun1,roscar);
	ros::Timer timer2=nh.createTimer(ros::Duration(1.0/pub_hz),&RosCarNode::fun2,roscar);
	roscar->Setup();
	roscar->spin();
	return 0;
}
