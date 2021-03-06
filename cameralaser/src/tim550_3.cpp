#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "tim550.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include<arpa/inet.h>
#define MAXLINE 80
#define MAXMSG 4096
#define SERV_PORT 8000

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
                    LDEBUG("debug %d\n",global_debug++);

        bzero(&servaddr, sizeof(servaddr));//clear it 
                    LDEBUG("debug %d\n",global_debug++);

        servaddr.sin_family = AF_INET;//config

        inet_pton(AF_INET, ip, &servaddr.sin_addr);//  config the ip 
                    LDEBUG("debug ip %d\n",global_debug++);

 //       servaddr.sin_port = htons(SERV_PORT);// config the port
      //  sscanf(port, "%d", &server_port);
        server_port = atoi(port);
                    LDEBUG("debug sscanf %d\n",global_debug++);
        servaddr.sin_port = htons(server_port);// config the port
                     LDEBUG("debug htons%d\n",global_debug++);

        ret =connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));
                     LDEBUG("debug %d\n",global_debug++);
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
    ADEBUG("send: %s \n",str_snd);         
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
	Y[i]=dist*sin((i-45)*3.1415926/180);
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


double door_rate(double start_num,double end_num,int*LC_R, int*LC_G, int*LC_B){
	double door_point = 0; double rate_door = 0;
    for (int i = start_num; i < end_num; i++){
        if (LC_R[i] < 120 && LC_G[i] < 120 && LC_B[i] < 120  ){
			door_point++;
		}
	}
    rate_door = door_point / (end_num-start_num);
    return rate_door;
	}


