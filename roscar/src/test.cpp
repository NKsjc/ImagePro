#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc,char ** argv)
{
	ros::init(argc,argv,"test");
	ros::NodeHandle n;
	ros::Publisher test_pub=n.advertise<std_msgs::String>("test",100);
	ros::Rate loop(20);
	while(ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss<<"test for ";
		msg.data==ss.str();
		test_pub.publish(msg);
		ros::spinOnce();
		loop.sleep();
	}
	return 0;
}
