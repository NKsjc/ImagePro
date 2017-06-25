#ifndef _COMMON_DEF_H
#define _COMMON_DEF_H

#define PORT "/dev/ttyS2"

///波特率设置
#define BIT_RATE 38400

///系统配置
#define WORLD_DT 10   //控制周期
#define PI 3.1415926
///v w 速度转换  
#define WHEEL_DIS 0.44 //两轮间距

///
#define ANGLE 8
#define ANGLE2 -8

//
#define frame_id_odom "odom"
#define frame_id_base_link "base_link"
///
#define RADIUS 0.105

///调试接口
#define LDEBUG(fmt,args...) //printf(fmt, ##args)
#define PDEBUG(fmt,args...) // printf(fmt, ##args)
#define ADEBUG(fmt,args...)  printf(fmt, ##args)

#endif
