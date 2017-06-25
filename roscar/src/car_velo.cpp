#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <semaphore.h>

#include <termios.h>

#include "common_def.h"
#include "car_velo.h"
#include "rs232.h"//38400
//send &receive count 
int Car_Velo_Recv = 0;
int Car_Velo_Snd = 0;
int Car_Status_Snd = 0;
int Car_Status_Recv = 0;

//last time the angle of wheel
int last_l_wheel=0;
int last_r_wheel=0;
float last_l_wheel_agl=0;
float last_r_wheel_agl=0;
float l_agl = 0;
float r_agl = 0;
int l_count = 0;//quanshu
int r_count = 0;//quanshu
float x=0;
float y=0;
float theta=0;
/*    term_attr.c_cc[VMIN] = 1;
    term_attr.c_cc[VTIME] = 1;*/
int read_wheel_data(int fd, char *buf, int timeout){
	int ret_len = 0, rd_len = 0;
	char rbuf[100];
	memset(rbuf, 0, sizeof(rbuf));
	//0
	int uTryTimes = 1000;
	while(timeout --){
		usleep(uTryTimes);
		rd_len = read(fd, rbuf, 1);
		if(rd_len > 0) break;
		if(timeout == 0) return -1;
	}
	if(rd_len !=1 || rbuf[0] != V_RECV_MASK)
		return ret_len;
	buf[0] = rbuf[0];
	ret_len ++;
	char crc = 0;
	while(ret_len < V_RECV_LEN){ 
			rd_len = read(fd, rbuf, 1);          //100ms 内无字符返回0
			if(rd_len !=1)
				return ret_len;
			buf[ret_len] = rbuf[0];
			crc  = crc ^ rbuf[0];
			ret_len ++;
	}
	//1 位CR (无作用待去除)
	rd_len = read(fd, rbuf, 1);
	if(rd_len !=1)
		return ret_len;
	ret_len ++;
	return ret_len;
}

int car_velo_init(const char *device){
    int fd = serial_init(device);
    Car_Velo_Recv = 0;
    Car_Velo_Snd = 0;
    return fd;
}
int car_get_sended(void){
    return Car_Velo_Snd;
}

int car_get_recved(void){
    return Car_Velo_Recv;
}
float car_get_x(void)
{
	return x;
}

float car_get_y(void)
{
	return y;
}

float car_get_theta(void)
{
	return theta;
}

int car_vw_outin(int fd,FILE *fp, char nIndex, float fVelo_V, float fVelo_W, char *p_nOIndex, float     * p_fX, float  * p_fY, float *  p_fARad,char p_l1,char p_l2,char p_r1,char p_r2,int l,int r)
{

    char str_out[V_SND_LEN],str_in[100];
    char rec_flag=0;
    int i, len, ret, timeout = 5;
    
    float fVeloLeft = 0.5 * (2.0 * fVelo_V - fVelo_W * WHEEL_DIS);
    float fVeloRight = 2.0 * fVelo_V -fVeloLeft;
    
    int nVeloLeft =  (fVeloLeft * VELO_TRANS_PER_INT / VELO_TRANS_PER_FLOAT); 
    int nVeloRight =  (fVeloRight * VELO_TRANS_PER_INT / VELO_TRANS_PER_FLOAT); 
	
	char LVelo0 = nVeloLeft;
	char LVelo1 = nVeloLeft>>8;

	char RVelo0 = nVeloRight;
	char RVelo1 = nVeloRight>>8;

	memset(str_in, 0, sizeof(str_in));
	memset(str_out, 0, sizeof(str_out));
	//LDEBUG("%hhx %hhx %hhx %hhx\n", LVelo0, LVelo1, RVelo0, RVelo1);
    str_out[0] = V_SND_MASK;
    str_out[1] = 0x00;    
    str_out[2] = 0x00;
        
    str_out[3] = 0xF0;   //
    str_out[4] = nIndex; //位置字节  
    
    str_out[5] = LVelo0;  //左速度 L 512 : 1km/h
    str_out[6] = LVelo1 & 0xFF;  //H

    str_out[7] = RVelo0;  //右速度 L 512 : 1km/h
    str_out[8] = RVelo1 & 0xFF;  //H
  	
	ADEBUG("%hhx %hhx %hhx %hhx %hhx\n",str_out[4], str_out[5], str_out[6], str_out[7], str_out[8]);
	//LDEBUG("%d\n",0x0733);
    char CRC = (str_out[0]) ^ (str_out[1]) ^ (str_out[2]) ^ (str_out[3]) ^ (str_out[4]) ^ (str_out[5]) ^ (str_out[6]) ^ (str_out[7]) ^ (str_out[8]); 

    str_out[9] = CRC;  //校验
    len = V_SND_LEN;    
    for(i = 0; i < len; i++){
        ADEBUG("[%hhx]\t",str_out[i]);	
    }
    tcflush(fd, TCIFLUSH | TCOFLUSH);
	//LDEBUG("send to %d:%10s\n", fd, str_out);
    ret = write_data(fd, str_out, len);
    //LDEBUG("len:%d ret:%d OK:%10s\n", len, ret, str_out);
    if(ret == len){
        Car_Velo_Snd ++;
    }else{
	    LDEBUG("adam read cfg fail:%d\n",ret);
    }
    
    //调试	
    //len = read_data(fd, str_in, V_RECV_LEN + 1, timeout);
    len = read_data_all(fd, str_in, V_RECV_LEN + 1 ,timeout) ;
    //len = read_wheel_data(fd, str_in, timeout);
    ADEBUG("Recv len: %d\n",len);
    /*for(i = 0; i < len; i++){
        ADEBUG("[%hhx]\t",str_in[i]);	
    }
    PDEBUG("\n");*/
    
   //有效数据
    char str_velo_in[V_RECV_LEN];
    memset(str_velo_in, 0, sizeof(str_velo_in));
    int valid_i = -1;
    for(i = 0; i < len; i++){
       if((str_in[i] & 0xFF) == 0xbb){
           valid_i = i;
           break;
       }
    }
   // if(len != 11) return -1;
    ADEBUG("valid:%d\n", valid_i);
    if((valid_i >= 0) && (len - valid_i) >= (V_RECV_LEN)){
        memcpy(str_velo_in, (str_in + valid_i), V_RECV_LEN);
    }else{
		return -1;
   }

    for(i = 0; i < V_RECV_LEN; i++){
        ADEBUG("[%hhx]\t", str_velo_in[i]);	
    }
    ADEBUG("\n");
   
    CRC = (str_velo_in[0]) ^ (str_velo_in[1]) ^ (str_velo_in[2]) ^ (str_velo_in[3]) ^ (str_velo_in[4]) ^ (str_velo_in[5]) ^ (str_velo_in[6]) ^ (str_velo_in[7]) ^ (str_velo_in[8]);
     ADEBUG("CRC: %hhx\n",CRC);
     if( ((str_velo_in[3] & 0xFF) == 0xF0) && (str_velo_in[9] == CRC) ){
   //if((str_velo_in[9] == CRC)){
      Car_Velo_Recv++;
    //  *p_nOIndex = (char)(str_velo_in[4]);
      //ADEBUG("CRC: %hhx\n",CRC);
      /*int x_loc = str_velo_in[2] * 256 + str_velo_in[1];
      int y_loc = str_velo_in[6] * 256 + str_velo_in[5];
      int a_loc = str_velo_in[8] * 256 + str_velo_in[7];*/
      double delta_l=0;
      double delta_r=0;
      //calculate the angle of wheel ]
      int tempchar;
      tempchar=str_velo_in[1];
      if(tempchar<0)
      {
		  tempchar+=256;
	  }
      int a_wheel_agl = str_velo_in[2] * 256 + tempchar;
      tempchar=str_velo_in[5];
      if(tempchar<0)
      {
		  tempchar+=256;
	  }
      int b_wheel_agl = str_velo_in[6] * 256 + tempchar;
      
      //daochu 
      p_l1=str_velo_in[1];p_l2=str_velo_in[2];
      p_r1=str_velo_in[5];p_r2=str_velo_in[6];
      l=a_wheel_agl;
     r=b_wheel_agl;
     fprintf(fp,"%d  %d  \n", l,r);
      ADEBUG("left coder(%d %d)\n",  l, r);
      //shiyanguocheng chuxian fushu
      if(a_wheel_agl<0)
      {	a_wheel_agl = 0x0FFF+a_wheel_agl; }
      if(b_wheel_agl<0)
      {	b_wheel_agl = 0x0FFF+b_wheel_agl; }
      
     /* float f_awheel_agl = a_wheel_agl * (360.0 / 0x0FFF);
      float f_bwheel_agl = 360.0 - b_wheel_agl * (360.0 / 0x0FFF);*/
      //raw data
      float l_wheel_agl = a_wheel_agl * (30.0 / 0x0FFF);
      float r_wheel_agl =  b_wheel_agl * (30.0 / 0x0FFF);
      ADEBUG("(%f %f)\n", l_wheel_agl, r_wheel_agl);
      //
	  if(Car_Velo_Recv>1)
	  {
		  //left
		  if(l_wheel_agl-last_l_wheel_agl  > ANGLE)
		  {
				delta_l= 30.0-l_wheel_agl  + last_l_wheel_agl;
				l_agl +=delta_l;
			}
		  if(l_wheel_agl-last_l_wheel_agl  < ANGLE2)
		  {
				//delta_l= 30.0+l_wheel_agl  - last_l_wheel_agl;				
				//l_agl -=delta_l;
				delta_l= -1.0*(30.0+l_wheel_agl  - last_l_wheel_agl);				
				l_agl +=delta_l;
			}
		  if((l_wheel_agl-last_l_wheel_agl  <= ANGLE)&&(l_wheel_agl-last_l_wheel_agl  >= ANGLE2))
		  {
				delta_l= -1.0*(l_wheel_agl  - last_l_wheel_agl);
				l_agl += delta_l;
			}
		   //right
		   if(r_wheel_agl-last_r_wheel_agl  > ANGLE)
		   {
				delta_r= -(30.0-r_wheel_agl  + last_r_wheel_agl);
				r_agl +=delta_r;
		  }
		  if(r_wheel_agl-last_r_wheel_agl  < ANGLE2)
			{	
				delta_r= 30.0+r_wheel_agl  - last_r_wheel_agl;
				r_agl += delta_r;
			}
		  if((r_wheel_agl-last_r_wheel_agl  <= ANGLE)&&(r_wheel_agl-last_r_wheel_agl  >= ANGLE2))
		  {
				delta_r= r_wheel_agl  - last_r_wheel_agl;
				r_agl += delta_r;
			}
			ADEBUG("(%f,%f)\n",l_agl,r_agl);
			
			//left
			if(l_agl>=360)
			{
				l_agl -=360;
				l_count++;
			}
			if(l_agl<0)
			{
				l_agl +=360;
				l_count--;
			}
			//right	
			if(r_agl>=360)
			{
				r_agl -=360;
				r_count++;
			}
			if(r_agl<0)
			{
				r_agl +=360;
				r_count--;
			}
	  }
	  ADEBUG("left count %d right count %d\n",l_count,r_count);
	  
	  //odom
	
	  x+=0.5*(delta_r+delta_l)/180*PI*RADIUS*cos(theta);
	  y+=0.5*(delta_r+delta_l)/180*PI*RADIUS*sin(theta);
	  theta+=(delta_r-delta_l)/180*PI*RADIUS/WHEEL_DIS;
	 // theta+=(delta_r-delta_l)/180*PI*RADIUS/WHEEL_DIS;
	  //theta's yueshu 
	   if(theta>=PI)
		{
			theta-=2*PI;
		}
		if(theta<=-PI)
		{
			theta+=2*PI;
		}
		  /*FILE *fp;
	  fp=fopen("tra.txt","a+");
	  if(fp==NULL)
			  ADEBUG("open file error");
		fprintf(fp,"%f  %f  %f\n",x,y,theta);
		fclose(fp);*/
	ADEBUG("x %f  y %f,theta %f\n",x,y,theta);
	/**p_fX=x;
	*p_fY=y;
	*p_fARad=theta;*/
	  //
	  last_l_wheel = a_wheel_agl;
	  last_l_wheel = b_wheel_agl;
	  last_l_wheel_agl  = l_wheel_agl;
	  last_r_wheel_agl = r_wheel_agl;
      //1mm
     /* 
      p_fX = x_loc * 0.001;
      p_fY = y_loc * 0.001;
      p_fARad = a_loc * 0.001;   
      */
      //LDEBUG("Recv Loc! (%d %f %f %f)\n",*p_nOIndex, *p_fX, *p_fY, *p_fARad);
      //fADEBUG(Fd_car, "%d %f %f %f \n",*p_nOIndex, *p_fX, *p_fY, *p_fARad);
      return 0;
   }
    
   LDEBUG("Send counter: %d Recv counter %d\n",Car_Velo_Snd,Car_Velo_Recv);
   return -1;
}
//[c0]	[c0]	[f8]	[c0]	[f0]	[bb]	[66]	[55]	[f0]	[00]	[55]	[66]	[48]	[03]	[00]	[0a]

//add the function of solve getting the status of robot 
//including the status of the battery & son on
int car_status_outin(int fd,char nIndex,float * robot_status,float* battery_status,float* cmd_status,float * motor1_status,float * motor2_status,float * led_status)
{
	char str_out[V_SND_LEN],str_in[100];
    char rec_flag=0;
    int i, len, ret, timeout = 5;
    
	memset(str_in, 0, sizeof(str_in));
	memset(str_out, 0, sizeof(str_out));
	//LDEBUG("%hhx %hhx %hhx %hhx\n", LVelo0, LVelo1, RVelo0, RVelo1);
    str_out[0] = V_SND_MASK;
    str_out[1] = 0x00;
    str_out[2] = 0x00;
    str_out[3] = 0x39;    
    str_out[4] = 0x00;
    str_out[5] = 0x00;
    str_out[6] = 0x00;
    str_out[7] = 0x00;
    str_out[8] = 0x00;
    //crc//校验
    char CRC = (str_out[0]) ^ (str_out[1]) ^ (str_out[2]) ^ (str_out[3]) ^ (str_out[4]) ^ (str_out[5]) ^ (str_out[6]) ^ (str_out[7]) ^ (str_out[8]); 
    str_out[9] = CRC;  
    
    tcflush(fd, TCIFLUSH | TCOFLUSH);
    len=V_SND_LEN;
    for(i=0;i<len;i++)
    {
		ADEBUG("[%hhx]\t",str_out[i]);
	}
    ret = write_data(fd, str_out, len);
    //LDEBUG("len:%d ret:%d OK:%10s\n", len, ret, str_out);
    if(ret == len){
        Car_Status_Snd ++;
        ADEBUG(" status send:%d\n",Car_Status_Snd);
    }else{
	    ADEBUG(" read status of robot failed:%d\n",ret);
    }
    
    //调试	
    //len = read_data(fd, str_in, V_RECV_LEN + 1, timeout);
    len = read_data_all(fd, str_in, V_RECV_LEN + 1 ,timeout) ;
    //len = read_wheel_data(fd, str_in, timeout);
    ADEBUG("Recv len: %d\n",len);
    /*for(i = 0; i < len; i++){
        ADEBUG("[%hhx]\t",str_in[i]);	
    }
    PDEBUG("\n");*/
    
   //有效数据
    char str_velo_in[V_RECV_LEN];
    memset(str_velo_in, 0, sizeof(str_velo_in));
    int valid_i = -1;
    for(i = 0; i < len; i++){
       if((str_in[i] & 0xFF) == 0xbb){
           valid_i = i;
           break;
       }
    }
   // if(len != 11) return -1;
    ADEBUG("valid:%d\n", valid_i);
    if((valid_i >= 0) && (len - valid_i) >= (V_RECV_LEN)){
        memcpy(str_velo_in, (str_in + valid_i), V_RECV_LEN);
    }else{
		return -1;
   }

    for(i = 0; i < V_RECV_LEN; i++){
        ADEBUG("[%hhx]\t", str_velo_in[i]);	
    }
    ADEBUG("\n");
   
    CRC = (str_velo_in[0]) ^ (str_velo_in[1]) ^ (str_velo_in[2]) ^ (str_velo_in[3]) ^ (str_velo_in[4]) ^ (str_velo_in[5]) ^ (str_velo_in[6]) ^ (str_velo_in[7]) ^ (str_velo_in[8]);
     ADEBUG("CRC: %hhx\n",CRC);
     if( ((str_velo_in[3] & 0xFF) == 0xF0) && (str_velo_in[9] == CRC) ){
		//if((str_velo_in[9] == CRC)){
      Car_Status_Recv++;
      /*int a_wheel_agl = str_velo_in[2] * 256 + str_velo_in[1];
      int b_wheel_agl = str_velo_in[6] * 256 + str_velo_in[5];
      ADEBUG("(%d %d)\n", a_wheel_agl, b_wheel_agl);
      
      float f_awheel_agl = a_wheel_agl * (360.0 / 0x0FFF);
      float f_bwheel_agl = 360.0 - b_wheel_agl * (360.0 / 0x0FFF);
      ADEBUG("(%f %f)\n", f_awheel_agl, f_bwheel_agl);
      //1mm
     /* 
      p_fX = x_loc * 0.001;
      p_fY = y_loc * 0.001;
      p_fARad = a_loc * 0.001;   
      
      LDEBUG("Recv Loc! (%d %f %f %f)\n",*p_nOIndex, *p_fX, *p_fY, *p_fARad);*/
  }
}
