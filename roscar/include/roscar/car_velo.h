#ifndef _CAR_VELO_H
#define _CAR_VELO_H
#include <stdio.h>
#define V_SND_MASK 0xAA
#define V_RECV_MASK 0xbb
#define V_SND_LEN 10
#define V_RECV_LEN 10
 
/// 速度转串口数据
#define VELO_TRANS_PER_INT 512  // 
#define VELO_TRANS_PER_FLOAT (1.0f / 3.6f) //m/s
#define LEN_SET 10

///初始化
extern int car_velo_init(const char *device);
///里程计清零

int car_get_sended(void);

int car_get_recved(void);

///
float car_get_x(void);

float car_get_y(void);

float car_get_theta(void);
///速度(加)
extern int car_vw_outin(int fd, FILE *fp,char nIndex, float fVelo_V, float fVelo_W, char *p_nOIndex, float * p_fX, float *p_fY, float *p_fARad,char p_l1,char p_l2,char p_r1,char p_r2,int l,int r);

///电量函数 & motor status
extern int car_status_outin(int fd,char nIndex,float * robot_status,float* battery_status,float* cmd_status,float * motor1_status,float * motor2_status,float * led_status);
#endif
