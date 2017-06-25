///调试接口
#ifndef COM_DEF_H
#define COM_DEF_H
#include <ros/ros.h>
#define SDEBUG(fmt,args...) printf(fmt, ##args)
#define PDEBUG(fmt,args...)  printf(fmt, ##args)
#define ADEBUG(fmt,args...)  printf(fmt, ##args)
#define PI_M 3.1415926

typedef struct{
  double v[3];
} d_pose;

typedef struct{
  int start;
  int end;
}TwoINT;

typedef struct{
  int points_num;
  float angle_increment;
  double* scan;//scan data and corresponding odom data
  d_pose odom;
  double (*ranges)[2];
  ros::Time rTime;
} d_frame;

#endif