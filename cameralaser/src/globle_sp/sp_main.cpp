#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iomanip>
#include "matrix_func.h"
#include "jiaodian.h"
#include "test1function.h"
using namespace std;
int main(int argc,char ** argv){
    tim550_t nav_dev;
        char * dev_ip="169.254.85.10"; //   169.254.135.234 192.168.3.111
        char *dev_port="2112";  //                  2112
         int ret=0 ;
        //fpoint_t mark_points[MAX_LOCATION];
        int mark_num =0;
        //flocate_t mylocate;
        //enum NA_MOD working_mod;
        //LDEBUG("debug %d\n",global_debug++);
        printf("hello word! \n");
        nav_dev.sockfd = tim550_connect(dev_ip, dev_port);
        if(nav_dev.sockfd <0 ){
                ADEBUG("Can't connect to %s %s\n",dev_ip,dev_port);
            return 0;
        }else{

                  ADEBUG("connect to %s %s\n",dev_ip,dev_port);
             }

        int loops = 0;
            int inifit = 0;
            int sp_num = 0;
            double Px0[NP] = { 0 }, Py0[NP] = { 0 };
            double dist_org[NP][NP] = { 0 }; double dist_loc[NP][NP] = { 0 };
      FILE *fp;
        fp=fopen("globle_xy.txt","w+");

while (1){
    loops++;
    int X[300] = { 0 }, Y[300] = { 0 };
ret =goto_mod(&nav_dev,X,Y);//获取激光点坐标
int EX[300] = { 0 };
        int EY[300] = { 0 };
        int num_D = 0;//有效激光点数目
        int big_part[300][3] = { 0 };
        int parts = 0;//分割部分数目
        ret = cut_part(X, Y, EX, EY, num_D, big_part, parts);//自适应阈值分割
        int corner_num[NP] = { 0 };
                int corn_num = 0;
                ret = get_corner_point(EX, EY, parts, big_part, corner_num, corn_num);//角点提取
                double  Px[NP] = { 0 }, Py[NP] = { 0 };

                        if (corn_num>=2&&inifit==0){
                            sp_num = corn_num;
                            inifit = 1;
                            for (int i = 0; i < corn_num; i++)
                            {
                                Px0[i] = EX[corner_num[i]];
                                Py0[i] = EY[corner_num[i]];
						fprintf(fp, "%f  %f  \n",Px0[i],Py0[i]);
                            }
                            fclose(fp);
                        }
                        if (inifit==1){
                            for (int i = 0; i < corn_num; i++)
                            {
                                Px[i] = EX[corner_num[i]];
                                Py[i] = EY[corner_num[i]];
                            }
                            memset(dist_loc,0,sizeof(dist_loc));
                            ret = cal_dist(Px0, Py0, sp_num, dist_org);
                            ret = cal_dist(Px, Py, corn_num, dist_loc);
                        }
                        int globle_id[NP] = { 0 }; int loc_id[NP] = { 0 };
                        double rt[4] = { 0 };
                        if (inifit==1)
                        {
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
                               // fprintf(fp1, "%f %f %f  \n", rt[2], rt[3], theta);
                                //std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            }
                        }

    }

}
