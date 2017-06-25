#include<iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "test1function.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include<arpa/inet.h>
#define MAXLINE 80
#define MAXMSG 4096
#define SERV_PORT 8000
using namespace std;

static int search_sections_num(char *str, char flag){
        int n=0;
        while(*str){
            if(*str==flag)
                n++;
            str++;
        }
        return n;
}

int tim550_connect(char *ip,  char *port){
   	 int global_debug=0;
        struct sockaddr_in servaddr;
//        char buf[MAXLINE];
        int sockfd, n, ret;
//        char *str;
        int server_port;
        
        sockfd = socket(AF_INET, SOCK_STREAM, 0);// get a fd
                 //   LDEBUG("debug %d\n",global_debug++);

        bzero(&servaddr, sizeof(servaddr));//clear it 
                 //   LDEBUG("debug %d\n",global_debug++);

        servaddr.sin_family = AF_INET;//config

        inet_pton(AF_INET, ip, &servaddr.sin_addr);//  config the ip 
                //    LDEBUG("debug ip %d\n",global_debug++);

 //       servaddr.sin_port = htons(SERV_PORT);// config the port
      //  sscanf(port, "%d", &server_port);
        server_port = atoi(port);
             //       LDEBUG("debug sscanf %d\n",global_debug++);
        servaddr.sin_port = htons(server_port);// config the port
               //      LDEBUG("debug htons%d\n",global_debug++);

        ret =connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
               //      LDEBUG("debug %d\n",global_debug++);
        if(ret >=0){
                return sockfd;
        }else{
                return -1;
        }
 /*       write(sockfd, str, strlen(str));
        
        n = read(sockfd, buf, MAXLINE);

        printf("Response from server:/n");

        write(STDOUT_FILENO, buf, n);

        close(sockfd);
*/
    return 0;
}


//收发信息
//send telegram and get data
int goto_mod(tim550_t *dev,int *X,int *Y){
//int goto_mod(tim550_t *dev, char mode){
    char str_snd[MAXMSG], str_recv[MAXMSG];
    int snd_n,  recv_n, ret;
    
    memset(str_snd, 0, MAXMSG);
    memset(str_recv, 0, MAXMSG);
    memset(X, 0, sizeof(X));
    memset(Y, 0, sizeof(Y));
            //snd_n = sprintf(str_snd+1,"sRN LMDscandata %x",mode);
            snd_n = sprintf(str_snd+1,"sRN LMDscandata");
            str_snd[0] = 0x02;
            str_snd[snd_n+1] = 0x03;
            str_snd[snd_n+2] = '\0';
            
            ret =write(dev->sockfd, str_snd, strlen(str_snd));
            usleep(1000);
            recv_n = read(dev->sockfd, str_recv, MAXMSG);
  //  ADEBUG("send: %s \n",str_snd);         
//    ADEBUG("goto mod recv: %s \n",str_recv); 
//读数校验
  if(search_sections_num(str_recv,0x03)<1){   //待完善
            recv_n = read(dev->sockfd, str_recv, MAXMSG);
//            ADEBUG("goto mod recv2: %s \n",str_recv); 
        }
 




  char raw_point_str[MAXMSG];
  memset(raw_point_str, 0, MAXMSG);
int r_num=0,d_num=0,s_num=0;
            for(r_num;r_num<strlen(str_recv);r_num++){
               if(str_recv[r_num]==32){
               s_num++;
              }
              if(s_num>=25&&s_num<=296){
              raw_point_str[d_num]=str_recv[r_num];
               d_num++;
              }
             } 
            //PDEBUG("raw_str: %s \n",raw_point_str);

char point_str[MAXMSG];
memset(point_str, 0, MAXMSG);
int i=0;
int dis_str[MAXMSG]={0};
memset(dis_str, 0, MAXMSG);
int dist=0;
     char c[] = " ";
    char *p = strtok(raw_point_str,c);
	sscanf(p,"%x",&dist);
        printf("we get %d points! \n",dist);
    p = strtok(NULL,c);
    while(p)
    {  
	sscanf(p,"%x",&dist);
//        printf("%d.%d\t",i,dist);
        X[i]=dist*cos((i-45)*3.1415926/180);
    Y[i]=dist*sin((i-45)*3.1415926/180)-0;
        p = strtok(NULL,c); 
	i++;
    }
     printf(" \n");

/*FILE *fp=fopen("test.txt","w");
int i=0;
for(i=0;i<strlen(raw_point_str);i++){
fprintf(fp,"%c",raw_point_str[i]);
 }
            fclose(fp);*/
 
        return 0;
} 




int xy_to_uv(int*X, int*Y, int *U, int*V){
	//中间摄像头参数
	double r111 = 0.9993, r112 = 0.0083, r113 = -0.0364, r121 = -0.0362, r122 = -0.0250, r123 = -0.9990, r131 = -0.0097, r132 = 0.9997, r133 = -0.0246;
	//double r11 = 0.9994, r12 = 0.0083, r13 = 0.0334, r21 = 0.0336, r22 = -0.0250, r23 = -0.9991, r31 = -0.0075, r32 = 0.9997, r33 = -0.0252;
	double t1x = -5, t1y = 90, t1z = -20;
	int f1x = 1415, f1y = 1394, u10 = 986, v10 = 558;
	//右侧摄像头参数
	double r211 = -0.0005, r212 = 0.9999, r213 = -0.0135, r221 = -0.1289, r222 = -0.0134, r223 = -0.9916, r231 = -0.9917, r232 = 0.0012, r233 = 0.1289;
	double t2x = 40, t2y = -28, t2z = -122;
	int f2x = 514, f2y = 513, u20 = 314, v20 = 256;
	//左侧摄像头参数
	double r311 = 0.0685, r312 = 0.9974, r313 = 0.0208, r321 = 0.1491, r322 = -0.0309, r323 = 0.9883, r331 = 0.9864, r332 = -0.0646, r333 = -0.1508;
	double t3x = -70, t3y = -32, t3z = -120;
	double f3x = 514, f3y = 513, u30 = 314, v30 = 256;

	double xc = 0, yc = 0, zc = 0;

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
		if (i>90 && i <= 180){//中间摄像头
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
	return 1;

}

//得到激光点的RGB信息
int get_rgb(int*U, int*V, int*X, int*Y, IplImage* pFrame[3], int LC[6][300],int *LC_num){//LC_num[0]-LC_num[3]
    
	CvScalar s;
	//int LC_num = 0;
	//int LC1_num = 0, LC2_num = 0, LC3_num = 0;
	for (int i = 0; i < 270; i++){

		if (i <= 90){
			//if (U[i] < 0 || U[i] >= 1920 || V[i] < 0 || V[i] >= 1080){
			if (U[i] < 0 || U[i] >= 640 || V[i] < 0 || V[i] >= 480){
				continue;
			}
			s = cvGet2D(pFrame[1], V[i], U[i]);
        //printf("θ=%d,X=%d,Y=%d,U=%d,V=%d,R=%f, G=%f, B=%f\n", i - 45, X[i], Y[i], U[i], V[i], s.val[2], s.val[1], s.val[0]);//R G B
			LC[0][LC_num[0]] = i;
			 LC[1][LC_num[0]] = X[i];
			 LC[2][LC_num[0]] = Y[i];
			 LC[3][LC_num[0]] = s.val[2];
			 LC[4][LC_num[0]] = s.val[1];
			 LC[5][LC_num[0]] = s.val[0];

			LC_num[0]++;
			LC_num[1]++;
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
		if (i>90 && i <= 180){
			if (U[i] < 0 || U[i] >= 1920 || V[i] < 0 || V[i] >= 1080){
				//if (U[i] < 0 || U[i] >=640 || V[i] < 0 || V[i] >= 480){
				continue;
			}
			s = cvGet2D(pFrame[0], V[i], U[i]);
      //  printf("θ=%d,X=%d,Y=%d,U=%d,V=%d,R=%f, G=%f, B=%f\n", i - 45, X[i], Y[i], U[i], V[i], s.val[2], s.val[1], s.val[0]);//R G B
			 LC[0][LC_num[0]] = i;
			 LC[1][LC_num[0]] = X[i];
			 LC[2][LC_num[0]] = Y[i];
			 LC[3][LC_num[0]] = s.val[2];
			 LC[4][LC_num[0]] = s.val[1];
			 LC[5][LC_num[0]] = s.val[0];

			LC_num[0]++;
			LC_num[2]++;

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

			if (U[i] < 0 || U[i] >= 640 || V[i] < 0 || V[i] >= 480){
				continue;
			}
			s = cvGet2D(pFrame[2], V[i], U[i]);
      //  printf("θ=%d,X=%d,Y=%d,U=%d,V=%d,R=%f, G=%f, B=%f\n", i - 45, X[i], Y[i], U[i], V[i], s.val[2], s.val[1], s.val[0]);//R G B
			 LC[0][LC_num[0]] = i;
			 LC[1][LC_num[0]] = X[i];
			 LC[2][LC_num[0]] = Y[i];
			 LC[3][LC_num[0]] = s.val[2];
			 LC[4][LC_num[0]] = s.val[1]; LC[5][LC_num[0]] = s.val[0];

			LC_num[0]++;
			LC_num[3]++;

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

	return 1;
}

//寻找激光点连续且最多的一组
int find_continues_point(int LC[6][300],int* LC_num,int &start_p,int &end_p){
	int linear_cut[30][3] = { 0 };
	int linear = 0;
	int num_count = 1;
	//int start_p = 0, end_p = 0;
	for (int j = LC_num[1]; j < LC_num[1] + LC_num[2]; j++){
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
	}
	return 1;
}

//门上激光点提取
int point_on_door(int start_p, int end_p, int LC[6][300], int wall[3][300],int &wall_num){
	for (int i = start_p; i < end_p + 1; i++){
		if (LC[3][i]<140 && LC[4][i]<140 && LC[5][i] < 140){
			wall[0][wall_num] = LC[0][i];
			wall[1][wall_num] = LC[1][i];
			wall[2][wall_num] = LC[2][i];
			wall_num++;
		}
		//	else{ break; }
	}
	return 1;
}
//墙上激光点提取

int point_on_wall(int start_p, int end_p, int LC[6][300], int wall[3][300], int &wall_num){
	for (int i = start_p; i < end_p + 1; i++){//提取墙面信息
		if (LC[3][i]>140 && LC[4][i]>140 && LC[5][i] > 140){
			wall[0][wall_num] = LC[0][i];
			wall[1][wall_num] = LC[1][i];
			wall[2][wall_num] = LC[2][i];
			wall_num++;
		}
		//	else{ break; }
	}
	return 1;
}

//拟合直线并计算θ和d
//这里直线只有两次分割
int linefit_calculate(int wall[3][300], int wall_num, double &theta ,double &dist_wall){
	double K = 0;
	double A = wall[2][0] - wall[2][wall_num - 1];
	double B = wall[1][wall_num - 1] - wall[1][0];
	double C = -1 * B*wall[2][wall_num - 1] - A*wall[1][wall_num - 1];
	//计算直线上的点到直线的距离
	double *d_line = new double[wall_num];
	double ABC = 0; double AB = 0;
	for (int i = 0; i < wall_num; i++){
		ABC = A*wall[1][i] + B*wall[2][i] + C;
		AB = sqrt(A*A + B*B);
		if (AB != 0){
			d_line[i] = abs(ABC / AB);
		}
		//cout << d_line[i] <<endl;
	}
	int temp = 0;
	for (int i = 0; i < wall_num; i++){
		if (d_line[i]>temp){
			temp = d_line[i];
		}
	}
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
		if (A*A + B*B != 0){
			if (-1 * A*C >= 0){
				dist_wall = abs(C / sqrt(A*A + B*B));

			}
			else{
				dist_wall = 2400 - abs(C / sqrt(A*A + B*B));

			}
		}

	}

	else{//<500 墙上所有点在一条直线上
		if (B == 0){
			K = PI / 2;
		}
		else { K = atan(-A / B); }
		if (K < 0) K = K + PI;
		theta = 90 - K * 180 / PI;
		if (A*A + B*B>0){
			if (-1 * A*C >= 0){
				dist_wall = abs(C / sqrt(A*A + B*B));

			}
			else{
				dist_wall = 2400 - abs(C / sqrt(A*A + B*B));

			}
		}
	}

	delete[]d_line;
	return 1;
}
//判断摄像机视野内门的概率（这里RGB值待改进，为避免干扰，应为门颜色对应的范围）
double door_rate(double start_num, double end_num, int LC[6][300]){
	double door_point = 0; double rate_door = 0;
	for (int i = start_num; i < end_num; i++){
        if (LC[3][i] < 130 && LC[5][i] < 130 && LC[5][i] < 130){
			door_point++;
		}
	}
	cout << "door points " << door_point;
	rate_door = door_point / (end_num - start_num);

	return rate_door;

}


//右侧空走廊检测
int  detect_passway(double start_num, double end_num, int LC[6][300]){
	double rate_passway = 0; double passway_point = 0; double dist = 0;
    for (int i = start_num; i < end_num; i++)
    {
		dist = sqrt(LC[1][i] * LC[1][i] + LC[2][i] * LC[2][i]);
		if (dist>3500){ passway_point++; }
	}
	rate_passway = passway_point / (end_num - start_num);
	if (rate_passway > 0.6){ return 1; }
	else{ return 0; }

}

//  避障和导航

int avoid_obstacles(int *X, int *Y, int threshold,int work_state,double &angle,double &length,int &start_p,int &end_p){
    //int threshold = 3000;
      int space_cut[300][3] = { 0 };
	int space_num = 0;
	double dist = 0;
    int num_count = 0;
    int temp=0;
    //int start_p = 0, end_p = 0;
  if(work_state==0){
		
    for (int i = 45; i < 45 + 180; i++){
    //for (int i = 0; i < 270; i++){
		dist = sqrt(X[i] * X[i] + Y[i] * Y[i]);
        if (dist > threshold||dist==0){
			num_count++;
            if(i==45+180-1){
                space_cut[space_num][0] = num_count-1;
                space_cut[space_num][1] = i;
                space_num++;
            }
		}
        else{
            if(num_count==0){continue;}
            space_cut[space_num][0] = num_count;
			space_cut[space_num][1] = i;
            num_count = 0;
			space_num++;
		}
	}
	 temp = 0;
	for (int i = 0; i < space_num; i++){
		if (space_cut[i][0]>temp)
			temp = space_cut[i][0];
	}
	for (int i = 0; i < space_num; i++){
		if (temp == space_cut[i][0]){
            start_p = space_cut[i][1] - space_cut[i][0];
			end_p = space_cut[i][1];
		}
	}
        if(start_p==45){start_p=start_p-1;}
}
//back
  if(work_state==1){
		
    for (int i = 45+30; i < 45 + 180; i++){
    //for (int i = 0; i < 270; i++){
		dist = sqrt(X[i] * X[i] + Y[i] * Y[i]);
        if (dist >5000||dist==0){
			num_count++;
            if(i==45+180-1){
                space_cut[space_num][0] = num_count-1;
                space_cut[space_num][1] = i;
                space_num++;
            }
		}
        else{
            if(num_count==0){continue;}
            space_cut[space_num][0] = num_count;
			space_cut[space_num][1] = i;
            num_count = 0;
			space_num++;
			cout<<"space_num="<<space_num<<'\t'<<"num_count="<<num_count<<'\t'<<'i='<<i<<endl;
		}
	}
	 temp = 0;
	for (int i = 0; i < space_num; i++){
		if (space_cut[i][0]>temp)
			temp = space_cut[i][0];
	}
	for (int i = 0; i < space_num; i++){
		if (temp == space_cut[i][0]){
            start_p = space_cut[i][1] - space_cut[i][0];
			end_p = space_cut[i][1];
		}
	}
        if(start_p==45+30){start_p=start_p-1;}
}




        length = sqrt((X[start_p] - X[end_p])*(X[start_p] - X[end_p]) + (Y[start_p] - Y[end_p])*(Y[start_p] - Y[end_p]));
        if((X[start_p]==0&&Y[start_p]==0)||(X[end_p]==0&&Y[end_p]==0)){
            length=6000;//test
            }
        length = length / 1000;
        if(work_state==0){
        if(length>5){end_p=end_p-15;}// don't turn left
	}
	      if(work_state==1){
        if(length>5){start_p=start_p+30;}// don't turn right
     }
	
	angle = (start_p + end_p) / 2;
    angle=135-angle;
    angle = angle*PI / 180;//1.2 is  factor


    //避障调节（斥力）
    double v_angle=0;
    double d_near=20000;
    double dist_a=0;
    for(int i=45;i<180+45;i++){
        dist_a = sqrt(X[i] * X[i] + Y[i] * Y[i]);
        if(dist_a<d_near&&dist_a>0)
        {	d_near=dist_a;
            }
    }
     for(int i=45;i<180+45;i++){
        dist_a = sqrt(X[i] * X[i] + Y[i] * Y[i]);
        if(dist_a==d_near)
        {	if (dist_a<500&&i<=135){v_angle=20*PI/180;
			}
        if (dist_a<500&&i>135){v_angle=-20*PI/180;}
        break;
            }
    }
    angle=angle+v_angle;
    return temp;
}





//提取有效激光点
/*int get_effective_point(int *X,int *Y,int EP[3][300],int start_p,int end_p,int&e_point){
    e_point = 0;
    for (int i = start_p; i < end_p; i++){
        if (X[i] == 0 && Y[i] == 0){ continue; }
        EP[0][e_point] = i;
        EP[1][e_point] = X[i];
        EP[2][e_point] = Y[i];
        e_point++;
    }
    return 1;
}*/

int get_effective_point(int *X, int *Y, int EX[300],int EY[300],int start_p, int end_p, int&e_point){
    e_point = 0;
    for (int i = start_p; i < end_p; i++){
        if (X[i] == 0 && Y[i] == 0){ continue; }
        EX[e_point] = X[i];
        EY[e_point] = Y[i];
        e_point++;
    }
    return 1;
}


void LineFitting(int x[], int y[], int size, double& theta, double&dist_wall)
{
    double A = 0;
    double B = 0;
    double xmean = 0;
    double ymean = 0;
    for (int i = 0; i < size; i++)
    {
        xmean += x[i];
        ymean += y[i];
    }
    xmean /= size;
    ymean /= size;

    double sumx2 = 0;
    double sumxy = 0;
    for (int i = 0; i < size; i++)
    {
        sumx2 += (x[i] - xmean) * (x[i] - xmean);
        sumxy += (y[i] - ymean) * (x[i] - xmean);
    }

    A = sumxy / sumx2;
    B = ymean - A*xmean;
    theta = atan(A);
    dist_wall = abs(B / sqrt(A*A + 1));
}




