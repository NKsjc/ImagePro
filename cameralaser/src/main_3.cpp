#include<iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "tim550_3.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//add ros header file
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace std;
//own header file for pragram
#include "tim550.h"
#include "parameter.h"
int main(int argc,char ** argv){
        ros::init(argc,argv,"cameralaser");
        ros::NodeHandle nh;
        ros::Publisher cmd_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
        //read parameter
        ParameterReader p("cameralaser_par.txt");
        //double PI_=atof(p.getData("PI").c_str());
        double DIS=atof(p.getData("DIS").c_str());
        double k1=atof(p.getData("k1").c_str());
        double k2=atof(p.getData("k2").c_str());
        double linear_x=atof(p.getData("linearx").c_str());
        double linear_a=atof(p.getData("linear_a").c_str());
        double angular_a=atof(p.getData("angular_a").c_str());
        int door_num=atoi(p.getData("door_num").c_str());
        double length = 0.175,v_max=1.5,last_angular=0.0,last_linear=0.0,last_d=0.0,last_theta=0.0;

        ADEBUG("parameter read :DIS %f  k1  %f k2 %f linearx %f,door_num  %d",DIS,k1,k2,linear_x,door_num);
        //save file path
        FILE *fp;
        fp=fopen("pointnew.txt","w+");

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
    if(nav_dev.sockfd <0 ){
            ADEBUG("Can't connect to %s %s\n",dev_ip,dev_port);
	    return 0;
    }else{
		
              ADEBUG("connect to %s %s\n",dev_ip,dev_port);
         }


IplImage* pFrame[4] = { NULL, NULL, NULL, NULL };

	CvCapture* pCapture[4] = { NULL, NULL, NULL, NULL };

	cvNamedWindow("Camera1", 0);
	cvNamedWindow("Camera2", 0);
	cvNamedWindow("Camera3", 0);
	//cvNamedWindow("Camera4",0);

	cvResizeWindow("Camera1", 300, 300);
	cvResizeWindow("Camera2", 300, 300);
	cvResizeWindow("Camera3", 300, 300);
	//cvResizeWindow("Camera4",300,300);

	cvMoveWindow("Camera1", 310, 0);
	cvMoveWindow("Camera2", 0, 0);
	cvMoveWindow("Camera3", 620, 0);
	//cvMoveWindow("Camera4",310,330);

	pCapture[0] = cvCaptureFromCAM(0);
	cvSetCaptureProperty(pCapture[0], CV_CAP_PROP_FRAME_WIDTH, 1920);
	cvSetCaptureProperty(pCapture[0], CV_CAP_PROP_FRAME_HEIGHT, 1080);
	pCapture[1] = cvCaptureFromCAM(1);
	//cvSetCaptureProperty(pCapture[1], CV_CAP_PROP_FRAME_WIDTH, 1920);
	//cvSetCaptureProperty(pCapture[1], CV_CAP_PROP_FRAME_HEIGHT, 1080);
	pCapture[2] = cvCaptureFromCAM(2);



	CvScalar s;
	int loops = 0;

double pass_door = 0;
	int last_rate_door = 0;
    double rateofdoor=0;
	int rate_door = 0;//假设从墙开始运动
	//外部帧循环
	while (1){
		loops++;

int X[300] = { 0 }, Y[300] = { 0 }, U[300] = { 0 }, V[300] = { 0 };
int LC_x[300] = { 0 }, LC_y[300] = { 0 }, LC_R[300] = { 0 }, LC_G[300] = { 0 }, LC_B[300] = { 0 };//存放融合之后的数据
//中间摄像头参数
double r111 = 0.9993, r112 = 0.0083, r113 = -0.0364, r121 = -0.0362, r122 = -0.0250, r123 = -0.9990, r131 = -0.0097, r132 = 0.9997, r133 = -0.0246;
double t1x = -5, t1y = 90, t1z = -20;
int f1x = 1415, f1y = 1394, u10 = 986, v10 = 558;
//右侧摄像头参数		
double r211 = -0.0005, r212 = 0.9999, r213 = -0.0135, r221 = -0.1289, r222 = -0.0134, r223 = -0.9916, r231 = -0.9917, r232 = 0.0012, r233 = 0.1289;
double t2x = 40, t2y = -28, t2z = -122;
int f2x = 514, f2y = 513, u20 = 314, v20 = 256;
//左侧摄像头参数
double r311 = 0.0685, r312 = 0.9974, r313 = 0.0208, r321 = 0.1491, r322 = -0.0309, r323 = 0.9883, r331 = 0.9864, r332 = -0.0646, r333 = -0.1508;
double t3x = -70, t3y = -32, t3z = -120;
int f3x = 514, f3y = 513, u30 = 314, v30 = 256;

double xc = 0, yc = 0, zc = 0;

 ret =goto_mod(&nav_dev,X,Y);//获取激光点坐标



	for (int i = 0; i < 270; i++){
			//for (i = 135; i < 270; i++){
			if (i <= 90){//右侧摄像头
			xc = r211*X[i] + r212*Y[i] + r213 * 0 + t2x;
			yc = r221*X[i] + r222*Y[i] + r223 * 0 + t2y;
			zc = r231*X[i] + r232*Y[i] + r233 * 0 + t2z;
			if (X[i] == 0 && Y[i] == 0){
				U[i] = 9999;
				V[i] = 9999;
			}
			else{
				U[i] = f2x*(xc / zc) + u20;
				V[i] = f2y*(yc / zc) + v20;
			}
		}
			if (i>90&&i<=180){//中间摄像头
				xc = r111*X[i] + r112*Y[i] + r113 * 0 + t1x;
				yc = r121*X[i] + r122*Y[i] + r123 * 0 + t1y;
				zc = r131*X[i] + r132*Y[i] + r133 * 0 + t1z;
				if (X[i] == 0 && Y[i] == 0){
					U[i] = 9999;
					V[i] = 9999;
				}
				else{
					U[i] = f1x*(xc / zc) + u10;
					V[i] = f1y*(yc / zc) + v10;
				}
			}
			if (i>180){//左侧摄像头
				xc = r311*X[i] + r312*Y[i] + r313 * 0 + t3x;
				yc = r321*X[i] + r322*Y[i] + r323 * 0 + t3y;
				zc = r331*X[i] + r332*Y[i] + r333 * 0 + t3z;
				if (X[i] == 0 && Y[i] == 0){
					U[i] = 9999;
					V[i] = 9999;
				}
				else{
					U[i] = f3x*(xc / zc) + u30;
					V[i] = f3y*(yc / zc) + v30;
				}
			}

		}


		pFrame[0] = cvQueryFrame(pCapture[0]);
		pFrame[1] = cvQueryFrame(pCapture[1]);
		pFrame[2] = cvQueryFrame(pCapture[2]);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~	
		int LC_num = 0;
		int LC1_num = 0, LC2_num = 0, LC3_num = 0;
		int LC[6][300] = { 0 };
		
		for (int i = 0; i < 270; i++){
		
			if (i <= 90){
				//if (U[i] < 0 || U[i] >= 1920 || V[i] < 0 || V[i] >= 1080){
					if (U[i] < 0 || U[i] >=640 || V[i] < 0 || V[i] >= 480){
					continue;
				}
				s = cvGet2D(pFrame[1], V[i], U[i]);
            //printf("θ=%d,X=%d,Y=%d,U=%d,V=%d,R=%f, G=%f, B=%f\n", i - 45, X[i], Y[i], U[i], V[i], s.val[2], s.val[1], s.val[0]);//R G B
				LC_x[LC_num] = X[i]; LC[0][LC_num] = i;
				LC_y[LC_num] = Y[i]; LC[1][LC_num] = X[i];
				LC_R[LC_num] = s.val[2]; LC[2][LC_num] = Y[i];
				LC_G[LC_num] = s.val[1]; LC[3][LC_num] = s.val[2];
				LC_B[LC_num] = s.val[0]; LC[4][LC_num] = s.val[1]; LC[5][LC_num] = s.val[0];

				LC_num++;
				LC1_num++;
				//在图像中显示激光点（拉长5像素以便观察）
				s.val[0] = 0;
				s.val[1] = 0;
				s.val[2] = 255;
				for (int j = 0; j < 5; j++){
					//if (U[i] + j < 1920)
						if (U[i] + j < 640)
						cvSet2D(pFrame[1], V[i], U[i] + j, s);//设定 (i,j) 像素
				}
			}
			
			//中间摄像头
			if (i>90&&i<=180){
				if (U[i] < 0 || U[i] >= 1920 || V[i] < 0 || V[i] >= 1080){
					//if (U[i] < 0 || U[i] >=640 || V[i] < 0 || V[i] >= 480){
					continue;
				}
				s = cvGet2D(pFrame[0], V[i], U[i]);
            //printf("θ=%d,X=%d,Y=%d,U=%d,V=%d,R=%f, G=%f, B=%f\n", i - 45, X[i], Y[i], U[i], V[i], s.val[2], s.val[1], s.val[0]);//R G B
				LC_x[LC_num] = X[i]; LC[0][LC_num] = i;
				LC_y[LC_num] = Y[i]; LC[1][LC_num] = X[i];
				LC_R[LC_num] = s.val[2]; LC[2][LC_num] = Y[i];
				LC_G[LC_num] = s.val[1]; LC[3][LC_num] = s.val[2];
				LC_B[LC_num] = s.val[0]; LC[4][LC_num] = s.val[1]; LC[5][LC_num] = s.val[0];

				LC_num++;
				LC2_num++;

				//在图像中显示激光点（拉长5像素以便观察）
				s.val[0] = 0;
				s.val[1] = 0;
				s.val[2] = 255;
				for (int j = 0; j < 5; j++){
					if (U[i] + j < 1920)
						//if (U[i] + j < 640)
						cvSet2D(pFrame[0], V[i], U[i] + j, s);//设定 (i,j) 像素
				}
			}
			//左侧摄像头（640P）
			if (i>180){
				
					if (U[i] < 0 || U[i] >=640 || V[i] < 0 || V[i] >= 480){
					continue;
				}
				s = cvGet2D(pFrame[2], V[i], U[i]);
           // printf("θ=%d,X=%d,Y=%d,U=%d,V=%d,R=%f, G=%f, B=%f\n", i - 45, X[i], Y[i], U[i], V[i], s.val[2], s.val[1], s.val[0]);//R G B
				LC_x[LC_num] = X[i]; LC[0][LC_num] = i;
				LC_y[LC_num] = Y[i]; LC[1][LC_num] = X[i];
				LC_R[LC_num] = s.val[2]; LC[2][LC_num] = Y[i];
				LC_G[LC_num] = s.val[1]; LC[3][LC_num] = s.val[2];
				LC_B[LC_num] = s.val[0]; LC[4][LC_num] = s.val[1]; LC[5][LC_num] = s.val[0];

				LC_num++;
				LC3_num++;

				//在图像中显示激光点（拉长5像素以便观察）
				s.val[0] = 0;
				s.val[1] = 0;
				s.val[2] = 255;
				for (int j = 0; j < 5; j++){
					//if (U[i] + j < 1920)
						if (U[i] + j < 640)
						cvSet2D(pFrame[2], V[i], U[i] + j, s);//设定 (i,j) 像素
				}
			}
			
		}

		cvShowImage("Camera1", pFrame[0]);
		cvShowImage("Camera2", pFrame[1]);
		cvShowImage("Camera3", pFrame[2]);


	int linear_cut[30][3] = { 0 };
		int linear = 0;
		int num_count = 1;
		int start_p = 0, end_p = 0;
        //for (int j = 0; j < LC_num; j++){
        for (int j = LC1_num; j < LC1_num+LC2_num; j++){
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
		for (int i = 0; i < linear; i++){
			if (linear_cut[i][0]>temp)
				temp = linear_cut[i][0];
		}
		for (int i = 0; i < linear; i++){
			if (temp == linear_cut[i][0]){
				start_p = linear_cut[i][1] + 1 - linear_cut[i][0];
				end_p = linear_cut[i][1];
			}
		}//激光点连续并且点数最多的一组
		//cout << "linear start num:" << start_p << "end num:" << end_p << endl;
		int wall_num = 0;
		int wall[3][100] = { 0 };
        double  start_num=start_p;
        double end_num=end_p;

       /* if(door_rate(start_num,end_num,LC_R,LC_G,LC_B)>0.7){

            for (int i = start_p; i < end_p + 1; i++){//提取 door
                if (LC[3][i]<140 && LC[4][i]<140 && LC[5][i] < 140){
                    wall[0][wall_num] = LC[0][i];
                    wall[1][wall_num] = LC[1][i];
                    wall[2][wall_num] = LC[2][i];
                    wall_num++;
                }

             }
        }*/
         //   else{
		for (int i = start_p; i < end_p + 1; i++){//提取墙面信息
            if (LC[3][i]>140 && LC[4][i]>140 && LC[5][i] > 140){
				wall[0][wall_num] = LC[0][i];
				wall[1][wall_num] = LC[1][i];
				wall[2][wall_num] = LC[2][i];
				wall_num++;
			}
			//	else{ break; }
            }
        // }
		//cout << wall[0][wall_num-1] << '\t' << wall[1][wall_num-1] << '\t' << wall[2][wall_num-1] << '\n';
		double A = wall[2][0] - wall[2][wall_num - 1];
		double B = wall[1][wall_num - 1] - wall[1][0];
		double C = -1 * B*wall[2][wall_num - 1] - A*wall[1][wall_num - 1];
		//计算直线上的点到直线的距离
		double *d_line = new double[wall_num];
		double ABC = 0; double AB = 0;
		for (int i = 0; i < wall_num; i++){
			ABC = A*wall[1][i] + B*wall[2][i] + C;
			AB = sqrt(A*A + B*B);
			d_line[i] = abs(ABC / AB);
			//cout << d_line[i] <<endl;
		}
		temp = 0;
		for (int i = 0; i < wall_num; i++){
			if (d_line[i]>temp){
				temp = d_line[i];
			}
		}
		double K = 0;
		double theta = 0;
		double dist_wall = 0;

		if (temp >= 500){//如果中间有点离该直线较远
			cout << "this line is devides" << endl;
			int break_n = 0;
			for (int i = 0; i < wall_num; i++){
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
			theta = 90 - K * 180 / PI;
            if(A*A+B*B!=0){
			if (-1 * A*C >= 0){
				dist_wall = abs(C / sqrt(A*A + B*B));

			}
			else{
				dist_wall = 2400 - abs(C / sqrt(A*A + B*B));

			}
		}
                    else{dist_wall=1200;}
    }




		else{//<500 墙上所有点在一条直线上
			if (B == 0){
				K = PI / 2;
			}
			else { K = atan(-A / B); }
			if (K < 0) K = K + PI;
			theta = 90 - K * 180 / PI;
             if(A*A+B*B!=0){
			if (-1 * A*C >= 0){
				dist_wall = abs(C / sqrt(A*A + B*B));

			}
			else{
				dist_wall = 2400 - abs(C / sqrt(A*A + B*B));

			}
		}
                 else{dist_wall=1200;}
    }
		delete[]d_line;

	cout << "theta:  " << theta << '\t' << "dist_wall:  " << dist_wall / 10 << endl;

		
		
		double LCn_num = LC1_num;
         rateofdoor= door_rate(0,LCn_num, LC_R, LC_G, LC_B);
        if(rateofdoor>0.7){rate_door=1;}
        if(rateofdoor<0.1){rate_door=0;}
		 if (rate_door - last_rate_door == 1){
			 pass_door++;
			 cout << "正在经过第 " << pass_door << "个门" << endl;
			
		 }
		 else{
			 cout << "正在从第" << pass_door << "个门到第" << pass_door + 1 << "个门" << endl;
		 }
		 last_rate_door = rate_door;//更新
         // fprintf(fp,"%f  %f\n",theta,dist_wall );
         /*control model w=factor*(k1*theta+k2*delta)
          *factor=linear_x/v_max
          *Author:  Jianchao Song
          * */
          if (cvWaitKey(5) == 27)
              break; //触发ESC键退出循环，读入数据停止
          geometry_msgs::Twist twist;
         if(door_num!=pass_door)
         {
              theta=theta/180*PI;
               // ROS_INFO("wsdafghasdhga sbglashdfg\n");
              //if the backforwad is wrong ,what should i do
              //like d & theta are error
             /* if(loops!=1)
              {

                  if(fabs(dist_wall-last_d)>0.5)
                  {
                      dist_wall=last_d;
                  }
                  if(fabs(theta-last_theta)>0.8)
                      theta=last_theta;
              }*/
              double d=DIS-dist_wall/1000.0+length*sin(theta);
              twist.angular.z = (k1*theta+k2*d)*linear_x/v_max;



              //increase
            /* if( (twist.angular.z-last_angular)>=angular_a)
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
             }*/
              //linear_x pinghuajiasu
             //solve problem decrease
             if((linear_x-last_linear)>=linear_a)
             {
                 twist.linear.x = last_linear+linear_a;
             }
             else
             {
                  twist.linear.x=linear_x;
             }
               twist.linear.x=linear_x;
             //twist.angular.z=last_angular;
              cmd_pub.publish(twist);
              fprintf(fp,"%f  %f  %f  %f\n",theta,d,twist.linear.x,twist.angular.z );
              last_angular=twist.angular.z;
              last_d=dist_wall;
              last_theta=theta;
            }
         else
         {
             cvReleaseCapture(&pCapture[0]);
             cvReleaseCapture(&pCapture[1]);
             cvReleaseCapture(&pCapture[2]);
             cvDestroyAllWindows();
            sleep(1);
            //rotate modle ,by experience
            ROS_INFO("xuanzhhuan");

             ros::Rate loopRate(100);
             int rate=205;//jingyan shuju
             twist.linear.x= 0.05;
             twist.angular.z = -0.5;
             double goal_angle = 3.14/2;
             int   ticks = int(goal_angle * rate);
             for(int t=0; t<ticks; t++)
             {
                     cmd_pub.publish(twist);
                     //ROS_INFO("pub sudu");
                     loopRate.sleep();
             }
             twist.linear.x=0;
             twist.angular.z=0;
             cmd_pub.publish(twist);
             return 0;
}


	}

	cvReleaseCapture(&pCapture[0]);
	cvReleaseCapture(&pCapture[1]);
	cvReleaseCapture(&pCapture[2]);

	cvDestroyAllWindows();
ros::spin();
return 0;
}




