#include <iostream>
#include <string.h>

#include <ros/ros.h>
//include <ros/
#include "lidar_tracking/motion_detecting.h"

int main(int argc,char** argv)
{
  ros::init(argc,argv,"lidar_tracking");
  
  SDEBUG("hahaha it's working\n");
  
  ros::NodeHandle nh;
  motion_detecting * m_d;
  m_d=new motion_detecting(nh);
  
  //motion_detecting m(nh);
  ros::spin();
  return 0;
}
