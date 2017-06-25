#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "test1function.h"
#include "matrix_func.h"
#include "jiaodian.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

//add ros header file
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "parameter.h"
using namespace std;

int main(int argc,char ** argv){
    ros::init(argc,argv,"cameralaser");
               ros::NodeHandle nh;
               ros::Publisher cmd_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
               //read parameter
               //juedingllujing   ～/catkin_ws  can not
               string file="/home/gqy/catkin_ws/param/cameralaser_par.txt";
               ParameterReader p(file);
               //double PI_=atof(p.getData("PI").c_str());
               double DIS=atof(p.getData("DIS").c_str());
               double k1=atof(p.getData("k1").c_str());
               //double k2=atof(p.getData("k2").c_str());
               double  k11= atof(p.getData("k11").c_str());
               double k22=atof(p.getData("k22").c_str());
               double linear_x=atof(p.getData("linearx").c_str());
               double linear_a=atof(p.getData("linear_a").c_str());
               double angular_a=atof(p.getData("angular_a").c_str());
               int door_num=atoi(p.getData("door_num").c_str());
               double robot_dis=atof(p.getData("robot_dis").c_str());
               int left_door=atoi(p.getData("left_door").c_str());
               double length = 0.175,v_max=1.5,last_angular=0.0,last_linear=0.0,last_d=0.0,last_theta=0.0;
                geometry_msgs::Twist twist;
               ADEBUG("parameter read :DIS %f  k1  %f k22 %f linearx %f,door_num  %d",DIS,k1,k22,linear_x,door_num);
               //save file path
				FILE *fp;
               fp=fopen("pointnew.txt","w+");
               FILE *fp0;
               fp0=fopen("/home/gqy/catkin_ws/src/cameralaser/src/globle_sp/globle_xy.txt","r");

               FILE *fp1;
               fp1=fopen("/home/gqy/catkin_ws/src/cameralaser/src/globle_sp/globle_xy1.txt","r");

               
		tim550_t nav_dev;
        char * dev_ip="169.254.85.10"; //   169.254.135.234 192.168.3.111
        char *dev_port="2112";  //                  2112
         int ret=0 ;

        int mark_num =0;

        nav_dev.sockfd = tim550_connect(dev_ip, dev_port);
        if(nav_dev.sockfd <0 ){
                ADEBUG("Can't connect to %s %s\n",dev_ip,dev_port);
            return 0;
        }else{

                  ADEBUG("connect to %s %s\n",dev_ip,dev_port);
             }
             
             
             
              int loops = 0;
        //double Px[300] = { 0 }, Py[300] = { 0 };

        double right_pass_door = 0;
        int last_right_rate_door = 0;
        int right_rate_door = 0;//从右摄像头墙开始计数

            double left_pass_door = 0;
        int last_left_rate_door = 0;
        int left_rate_door = 0;//从left摄像头墙开始计数

        double rateofdoor = 0;
        int det_passway = 0;
         int det_lift=0;
         int  work_state=0;
         int count_left_door=0;
         
         	int sp_num = 0;
	double Px0[NP] = { 0 }, Py0[NP] = { 0 };
	 char buf[1024]; 
	 printf("i am ok 3\n");
	while(fgets(buf,1024,fp0) != NULL){
		 sp_num++;
		 }
		  fclose(fp0);
		  fp0=fopen("/home/gqy/catkin_ws/src/cameralaser/src/globle_sp/globle_xy.txt","r");
		  //sp_num=7;
	 for(int i=0;i<sp_num;i++){
			 fscanf(fp0,"%lf  %lf",&Px0[i],&Py0[i]);
			 }
			 cout<<NP<<endl;
			 fclose(fp0);
			 //
			 double Px1[NP] = { 0 }, Py1[NP] = { 0 };
			 int sp_num1=0;
			 while(fgets(buf,1024,fp1) != NULL){
		 sp_num1++;
		 }
		  fclose(fp1);
		  fp1=fopen("/home/gqy/catkin_ws/src/cameralaser/src/globle_sp/globle_xy1.txt","r");
		  //sp_num=7;
	 for(int i=0;i<sp_num1;i++){
			 fscanf(fp1,"%lf  %lf",&Px1[i],&Py1[i]);
			 }
			 cout<<NP<<endl;
			 fclose(fp1);
			 
	double dist_org[NP][NP] = { 0 }; 
	double dist_loc[NP][NP] = { 0 };
int loops_count=0;
        while (1){
            start_work:
            loops++;
            int X[300] = { 0 }, Y[300] = { 0 }, U[300] = { 0 }, V[300] = { 0 };

            ret =goto_mod(&nav_dev,X,Y);//获取激光点坐标
               /* for(int i=0;i<270;i++){
                fprintf( fp2,"%d    %d      %d       \n",i,X[i],Y[i]);

            }*/


            int LC[6][300] = { 0 };
            int LC_num[4] = { 0 };

		
            double angle = 0, length = 0;
            int start_p=0,end_p=0;
            int EX[300] = { 0 }, EY[300] = { 0 }, e_piont = 0;
            double theta = 0, dist_wall = 0;
            int threshold =3000;
            double distance=0;
            int d1=0,d2=0;
            ret=avoid_obstacles(X, Y,threshold, work_state,angle, length,start_p,end_p);//输出为阈值内的空点数目
            //近距离障碍斥力对angle的影响
            
            
            d1=sqrt(X[start_p]*X[start_p]+Y[start_p]*Y[start_p]);
            d2=sqrt(X[end_p]*X[end_p]+Y[end_p]*Y[end_p]);
            
            if (d1==0)d1=6000;
            if(d2==0)d2=6000;
            
            double LC1_num = LC_num[1];
            double LC2_num = LC_num[2];
            double LCn_num = LC_num[0];
            
     //检测门数
         int num_D = 0;//有效激光点数目
		int big_part[300][3] = { 0 };
		int parts = 0;//分割部分数目
		ret = cut_part(X, Y, EX, EY, num_D, big_part, parts);//自适应阈值分割     
        int corner_num[NP] = { 0 };
		int corn_num = 0;
		ret = get_corner_point(EX, EY, parts, big_part, corner_num, corn_num);//角点提取
        double  Px[NP] = { 0 }, Py[NP] = { 0 };
        for (int i = 0; i < corn_num; i++)
			{
				Px[i] = EX[corner_num[i]];
				Py[i] = EY[corner_num[i]];
			}
			memset(dist_loc,0,sizeof(dist_loc));
			memset(dist_org,0,sizeof(dist_org));
			ret = cal_dist(Px1, Py1, sp_num1, dist_org);
			ret = cal_dist(Px, Py, corn_num, dist_loc);
			int globle_id[NP] = { 0 }; int loc_id[NP] = { 0 };
		int simular_num = simularfunc(corn_num, sp_num, dist_org, dist_loc, globle_id, loc_id); 
		if(simular_num>2){cout<<"near the door"<<endl;}
		if (simular_num>2&&abs(loops-loops_count)>150&&right_pass_door<8)  
		{
				loops_count=loops;
				right_pass_door++;
			}
			


     //检测电梯
    distance=sqrt(X[135]*X[135]+Y[135]*Y[135])/1000;
     if(length>4.7&&length<4.9&&distance>9.5){
             det_lift=1;

         }
    if(length>4.7&&length<4.9&&distance<9.3&& det_lift==1){
            right_pass_door =8 ;
         }
    if(right_pass_door==8){
            cout << "到达电梯" <<endl;
        }
    ////////////////////////////////////////



        printf("start_p %d  end_p  %d angle,%f,length %f,distance %f\n",start_p,end_p,angle,length,distance);
        //fprintf(fp,"%f             %f   \n",angle,length);
        
  //new4.17
 if(work_state==1){
		/*int num_D = 0;//有效激光点数目
		int big_part[300][3] = { 0 };
		int parts = 0;//分割部分数目
		ret = cut_part(X, Y, EX, EY, num_D, big_part, parts);//自适应阈值分割     
        int corner_num[NP] = { 0 };
		int corn_num = 0;
		ret = get_corner_point(EX, EY, parts, big_part, corner_num, corn_num);//角点提取
        double  Px[NP] = { 0 }, Py[NP] = { 0 };
        for (int i = 0; i < corn_num; i++)
			{
				Px[i] = EX[corner_num[i]];
				Py[i] = EY[corner_num[i]];
			}*/
			memset(dist_org,0,sizeof(dist_org));
			memset(dist_loc,0,sizeof(dist_loc));
			ret = cal_dist(Px0, Py0, sp_num, dist_org);
			ret = cal_dist(Px, Py, corn_num, dist_loc);
			int globle_id[NP] = { 0 }; int loc_id[NP] = { 0 };
		double rt[4] = { 0 };
		int simular_num = simularfunc(corn_num, sp_num, dist_org, dist_loc, globle_id, loc_id);
			if (simular_num < 3){ printf("simular_num %d is not enough \n", simular_num); }
			if (simular_num >= 3){
				ret = caculate_RT1(simular_num, globle_id, loc_id, Px0, Py0, Px, Py, rt);
				double theta = 0;
				if (rt[0] >= 0){
					if (rt[1] >= 0){
						theta = atan(rt[1] / rt[0]) * 180 / PI;
					}
					if (rt[1] < 0){
						theta = atan(rt[1] / rt[0]) * 180 / PI + 360;
					}
				}
				if (rt[0] < 0){
					if (rt[1] >= 0){
						theta = atan(rt[1] / rt[0]) * 180 / PI + 180;
					}
					if (rt[1] < 0){
						theta = atan(rt[1] / rt[0]) * 180 / PI + 180;
					}
				}
				//int xt = rt[2], yt = rt[3];
				cout << "simular_num " << simular_num << endl;
				cout << fixed << setprecision(1) << "x=" << rt[2] << '\t';
				cout << fixed << setprecision(1) << "y=" << rt[3] << '/t' << "       theta=" << theta << endl;
				//fprintf(fp1, "%f %f %f  \n", rt[2], rt[3], theta);
					if(sqrt(rt[2]*rt[2]+rt[3]*rt[3])<300){
				cout<<"to the point"<<endl;
				angle=45*PI/180;
				}
			}
		
	}	

            /*control model w=factor*(k1*theta+k2*delta)
                      *factor=linear_x/v_max
                      *Author:  Jianchao Song
                      * */

                      if (cvWaitKey(33) == 27)
                          break; //触发ESC键退出循环，读入数据停止

                     if(work_state==0&&(door_num!=right_pass_door))
                     {
                       // twist.linear.x=linear_x;
                         if(loops==1)
                             twist.linear.x =0;
                         //test need  stop
                         if(length<=robot_dis)
                         {
                             twist.linear.x=0;
                             twist.angular.z=0;
                             cmd_pub.publish(twist);
                         }
                         else
                         {
                             twist.angular.z=k1*angle*twist.linear.x/0.5;
                             twist.linear.x+=linear_a;
                             //set the max speed
                             if(twist.linear.x>linear_x)
                             {
                                 twist.linear.x = linear_x;
                             }
                              cmd_pub.publish(twist);
                             // fprintf(fp,"%f  %f  %f  %f\n",theta,d,twist.linear.x,twist.angular.z );
                              fprintf(fp,"%f       %f      %f         %f   \n",angle,length,twist.linear.x ,twist.angular.z);
                              last_angular=twist.angular.z;
                        }
                     }
                     if(work_state==0 && door_num==right_pass_door)
                     {
                            right_pass_door++;
                            work_state=1;
                            loops=0;
                            usleep(500);
                            //rotate modle ,by experience
                            ROS_INFO("xuanzhhuan");
                             ros::Rate loopRate(100);
                             int rate=212;//jingyan shuju
                             twist.linear.x= 0.01;
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
                             sleep(4);
                             twist.linear.x= 0.01;
                             twist.angular.z = -0.5;
                             for(int t=0; t<ticks; t++)
                             {
                                     cmd_pub.publish(twist);
                                     //ROS_INFO("pub sudu");
                                     loopRate.sleep();
                             }
                             sleep(1);
                    }

                    if (work_state==1&& left_pass_door!=left_door)
                    {
                        if(loops==1)
                             twist.linear.x =0;
                         //test need  stop
                         if(length<=robot_dis)
                         {
                             twist.linear.x=0;
                             twist.angular.z=0;
                             cmd_pub.publish(twist);
                         }
                         else
                         {
                             twist.angular.z=k1*angle*twist.linear.x/0.5;
                             twist.linear.x+=linear_a;
                             //set the max speed
                             if(twist.linear.x>linear_x)
                             {
                                 twist.linear.x = linear_x;
                             }
                              cmd_pub.publish(twist);
                             // fprintf(fp,"%f  %f  %f  %f\n",theta,d,twist.linear.x,twist.angular.z );
                              fprintf(fp,"%f       %f      %f         %f   \n",angle,length,twist.linear.x ,twist.angular.z);
                              last_angular=twist.angular.z;
                        }
                    }

                    if (work_state==1 && left_pass_door==left_door)
                    {
                             ROS_INFO("xuanzhhuan");
                             work_state=2;
                             ros::Rate loopRate(100);
                             int rate=215;//jingyan shuju
                             twist.linear.x= 0.01;
                             twist.angular.z = 0.5;
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


                         //cvReleaseCapture(&pCapture[0]);
                        // cvReleaseCapture(&pCapture[1]);
                        // cvReleaseCapture(&pCapture[2]);
                        // cvDestroyAllWindows();

                         //return 0;
                           if (cvWaitKey(33) == 27)
                                break; //触发ESC键退出循环，读入数据停止


        }



}
