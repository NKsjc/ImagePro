/*
 * Author :Jianchao Song
 * For laser conmunite
 * */
#pragma once

#include <netinet/in.h>

typedef struct
{
	int socketfd;
}laser_fd;
class LaserConnect
{
public:
	int global_debug=0;
	struct sockaddr_in servaddr;
	LaserConnect(char *ip,char * port)
	{
        int sockfd, n, ret;
        int server_port;       
        sockfd = socket(AF_INET, SOCK_STREAM, 0);// get a fd
                    LDEBUG("debug %d\n",global_debug++);

        bzero(&servaddr, sizeof(servaddr));//clear it 
                    LDEBUG("debug %d\n",global_debug++);

        servaddr.sin_family = AF_INET;//TCP/IP config

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
    return 0;
	}
};
