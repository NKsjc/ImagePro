//C++ STD header file
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

//opencv header file
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//own header file for pragram
#include "tim550.h"
#include "parameter.h"

//add ros header file
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//main function
int main(int argc,char ** argv){
    ros::init(argc,argv,"cameralaser");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
    //read parameter
    ParameterReader p("cameralaser_par.txt");
    //double PI_=atof(p.getData("PI").c_str());
    double DIS=atof(p.getData("DIS").c_str());
    double k1=atof(p.getData("k11").c_str());
    double k2=atof(p.getData("k22").c_str());
    double linear_x=atof(p.getData("linearx").c_str());
    double linear_a=atof(p.getData("linear_a").c_str());
    double angular_a=atof(p.getData("angular_a").c_str());
    double length = 0.175,v_max=1.5,last_angular=0.0,last_linear=0.0;
    ADEBUG("parameter read :DIS %f  k1  %f k2 %f linearx %f",DIS,k1,k2,linear_x);
    //save file path
    FILE *fp;
    fp=fopen("point.txt","w+");

    //激光雷达建立链接
    tim550_t nav_dev;
    char * dev_ip="169.254.85.10"; //   169.254.135.234 192.168.3.111
    char *dev_port="2112";  //                  2112 
     int ret=0 ;

    //fpoint_t mark_points[MAX_LOCATION];
    int mark_num =0;
    //flocate_t mylocate;
    //enum NA_MOD working_mod;
    //LDEBUG("debug %d\n",global_debug++);
    nav_dev.sockfd = tim550_connect(dev_ip, dev_port);
    if(nav_dev.sockfd <0 )
    {
            ADEBUG("Can't connect to %s %s\n",dev_ip,dev_port);
            return 0;
    }
    else
    {
              ADEBUG("connect to %s %s\n",dev_ip,dev_port);
    }
  // 摄像头建立链接
  //声明IplImage指针
  IplImage* pFrame = NULL;

 //获取摄像头
  CvCapture* pCapture = cvCreateCameraCapture(0);
  cvSetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_WIDTH,1920);
  cvSetCaptureProperty(pCapture,CV_CAP_PROP_FRAME_HEIGHT,1080);

  //创建窗口
  cvNamedWindow("video", 0);
  CvScalar s;
//激光和摄像头数据采集和交互

    double r11 = 0.9993, r12 = 0.0083, r13 = -0.0364, r21 = -0.0362, r22 = -0.0250, r23 = -0.9990, r31 = -0.0097, r32 = 0.9997, r33 = -0.0246;
    double tx = -5, ty = 90, tz = -20;
    int fx = 1415, fy = 1394, u0 = 986, v0 = 558;
    double xc = 0, yc = 0, zc = 0;
    int loop=0;

    while(1){
        loop++;
         printf("frame %d: \n",loop);
        int X[300]={0},Y[300]={0},U[300]={0},V[300]={0};
        int LC_x[300] = { 0 }, LC_y[300] = { 0 }, LC_R[300] = { 0 }, LC_G[300] = { 0 }, LC_B[300] = { 0 };//存放融合之后的数据
          //ret =goto_mod(&nav_dev,'\0');
        ret =goto_mod(&nav_dev,X,Y);

        for (int i = 0; i < 271; i++){//  激光点转换到像素点?
            xc = r11*X[i] + r12*Y[i] + r13 * 0 + tx;
            yc = r21*X[i] + r22*Y[i] + r23 * 0 + ty;
            zc = r31*X[i] + r32*Y[i] + r33 * 0 + tz;
            if (X[i] == 0 && Y[i] == 0){
                U[i] = 9999;
                V[i] = 9999;
            }
            else{
                U[i] = fx*(xc / zc) + u0;
                V[i] = fy*(yc / zc) + v0;
            }

        }
         pFrame=cvQueryFrame( pCapture );
            if(!pFrame)
                break;

        int LC_num = 0;
		int LC[6][100] = {0};
        //271 this  number is not good for every laser
		for (int i = 0; i < 271; i++){
			if (U[i] < 0 || U[i] >= 1920 || V[i] < 0 || V[i] >= 1080){
				continue;
			}
			s = cvGet2D(pFrame, V[i], U[i]);
	//		printf("θ=%d,X=%d,Y=%d,U=%d,V=%d,R=%f, G=%f, B=%f\n", i-45, X[i], Y[i],U[i],V[i] ,s.val[2], s.val[1], s.val[0]);//R G B
					LC_x[LC_num] = X[i]; LC[0][LC_num] = i;
					LC_y[LC_num] = Y[i]; LC[1][LC_num] = X[i];
					LC_R[LC_num] = s.val[2]; LC[2][LC_num] = Y[i];
					LC_G[LC_num] = s.val[1]; LC[3][LC_num] = s.val[2];
					LC_B[LC_num] = s.val[0]; LC[4][LC_num] = s.val[1]; LC[5][LC_num] = s.val[0];

			LC_num++;

			//在图像中显示激光点（拉长5像素以便观察）
			s.val[0] = 0;
			s.val[1] = 0;
			s.val[2] = 255;
			for (int j = 0; j < 5; j++){
				if (U[i] + j < 1920)
					cvSet2D(pFrame, V[i], U[i] + j, s);//设定 (i,j) 像素
			}
		}

   cvShowImage("video",pFrame);
	for(int i=0;i<271;i++){
	if(U[i]<0||U[i]>=1920||V[i]<0||V[i]>=1080){
          continue;}
        s = cvGet2D(pFrame, V[i],U[i]);
    //printf("num=%d,X=%d,Y=%d,B=%f, G=%f, R=%f\n",i,X[i],Y[i], s.val[0], s.val[1], s.val[2]);//B G R
}
	
	int linear_cut[30][3] = { 0 };
		int linear =0;
		int num_count = 1;
		int start_p = 0, end_p = 0;
		for (int j = 0; j < LC_num; j++){
			if (LC[0][j + 1] - LC[0][j] == 1){
				num_count++;
			}
			else{
				linear_cut[linear][0] = num_count;
				linear_cut[linear][1] = j;
				num_count = 1;
				linear++;
			}			
		}

		int temp = 0;
		for ( int i = 0; i < linear; i++){
			if (linear_cut[i][0]>temp)
				temp = linear_cut[i][0];			
		}
		for (int  i = 0; i < linear; i++){
			if (temp == linear_cut[i][0]){
				start_p = linear_cut[i][1] + 1 - linear_cut[i][0];
				end_p = linear_cut[i][1];
			}
		}//激光点连续并且点数最多的一组
		std::cout << "linear start num:" << start_p << "end num:" << end_p << std::endl;
		int wall_num = 0;
		int wall[3][100] = { 0 };
		for (int i = start_p; i < end_p+1; i++){//提取墙面信息
			if (LC[3][i]>110 && LC[4][i]>110 && LC[5][i] > 110){
				wall[0][wall_num] = LC[0][i];
				wall[1][wall_num] = LC[1][i];
				wall[2][wall_num] = LC[2][i];
					wall_num++;
			}	
		//	else{ break; }
		}
		//cout << wall[0][wall_num-1] << '\t' << wall[1][wall_num-1] << '\t' << wall[2][wall_num-1] << '\n';
		double A = wall[2][0] - wall[2][wall_num - 1];
		double B = wall[1][wall_num - 1] - wall[1][0];
		double C = -1 * B*wall[2][wall_num - 1] - A*wall[1][wall_num - 1];
		//计算直线上的点到直线的距离
		double *d_line = new double[wall_num];
		double ABC = 0; double AB = 0;
		for (int i = 0; i < wall_num; i++){
			ABC=A*wall[1][i] + B*wall[2][i] + C;
			AB = sqrt(A*A + B*B);
			d_line[i] = abs(ABC / AB);
			//cout << d_line[i] <<endl;
		}
		temp = 0;
		for ( int i = 0; i < wall_num; i++){
			if (d_line[i]>temp){
				temp = d_line[i];
			}	
		}
		double K = 0;
		double theta = 0;
		double dist_wall = 0;

		if (temp >= 500){//如果中间有点离该直线较远
			//cout << "this line is devides" << endl;
			int break_n = 0;
			for ( int i = 0; i < wall_num; i++){
				if (temp == d_line[i]){
					break_n = i;
				}
			}
			//取断点之前的部分做拟合
			A = wall[2][0] - wall[2][break_n - 1];
			B = wall[1][break_n - 1] - wall[1][0];
			C = -1 * B*wall[2][break_n - 1] - A*wall[1][break_n - 1];
			if (B == 0){
				K = PI / 2;
			}
			else { K = atan(-A / B); }


			if (K < 0){ K = K + PI; }
			theta = 90 - K * 180 /PI;
			 if (-1 * A*C >= 0){
				 dist_wall = abs(C / sqrt(A*A + B*B));
				 
			 }
			 else{
				 dist_wall = 2400-abs(C / sqrt(A*A + B*B));
			
			 }
		}

		else{//<500 墙上所有点在一条直线上
			if (B == 0){
				K = PI / 2;
			}
			else { K = atan(-A / B); }
			if (K < 0) K = K + PI;
			theta = 90 - K * 180 /PI;
			 if (-1 * A*C >= 0){
				 dist_wall = abs(C / sqrt(A*A + B*B));
				 
			 }
			 else{
				 dist_wall = 2400-abs(C / sqrt(A*A + B*B));
				
			 }
		}
		
		delete[]d_line;
    std::cout << "theta:  " << theta << '\t' << "dist_wall:  " << dist_wall / 10 << std::endl;
/*control model w=factor*(k1*theta+k2*delta)
 *factor=linear_x/v_max
 *Author:  Jianchao Song
 * */
     theta=theta/180*PI;
     geometry_msgs::Twist twist;
     double d=DIS-dist_wall/1000.0+length*sin(theta);
     twist.angular.z = (k1*theta+k2*d)*linear_x/v_max;
      twist.linear.x=linear_x;
     //increase
    /*if( (twist.angular.z-last_angular)>=angular_a)
    {
        twist.angular.z=last_angular+angular_a;
    }
    //decrease
    else
    {
            if( (twist.angular.z-last_angular)<=-1*angular_a)
            {
                twist.angular.z=last_angular-angular_a;
            }
            else
            {
                twist.angular.z=last_angular;
            }
    }
     //linear_x pinghuajiasu
    //solve problem decrease
    if((linear_x-last_linear)>=linear_a)
    {
        twist.linear.x = last_linear+linear_a;
    }
    else
    {
         twist.linear.x=linear_x;
    }*/
     cmd_pub.publish(twist);
     fprintf(fp,"%f  %f  %f  %f\n",theta,d,twist.linear.x,twist.angular.z );
     last_angular=twist.angular.z;
     char c=cvWaitKey(40);
      if(c==27)
          break;

  }
cvReleaseCapture(&pCapture);
cvDestroyWindow("video");
ros::spin();
return 0;
}

