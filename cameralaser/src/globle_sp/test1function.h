#ifndef _NAV350_H
#define _NAV350_H
#define  PI  3.14159
    #include <sys/types.h>
    #define MAX_LOCATION 10
    
    #define LDEBUG(fmt,args...) printf(fmt, ##args)
  
    #define PDEBUG(fmt,args...) printf(fmt, ##args)
    //#define PDEBUG(fmt,args...)
    
    #define ADEBUG(fmt,args...) printf(fmt, ##args)
   // #define PDEBUG(fmt,args...)

    
    
    typedef struct {
        int sockfd;
    }tim550_t;

    
    int tim550_connect(char *ip,  char *port);

    int goto_mod(tim550_t *dev,int *X,int *Y);

 /*int xy_to_uv(int*X, int*Y, int *U, int*V);
int get_rgb(int*U, int*V, int*X, int*Y, IplImage* pFrame[3], int LC[6][300], int *LC_num);
int find_continues_point(int LC[6][300], int *LC_num, int &start_p, int &end_p);
int point_on_door(int start_p, int end_p, int LC[6][300], int wall[3][300], int &wall_num);
int point_on_wall(int start_p, int end_p, int LC[6][300], int wall[3][300], int &wall_num);
int linefit_calculate(int wall[3][300], int wall_num, double &theta, double &dist_wall);
double door_rate(double start_num, double end_num, int LC[6][300]);
int  detect_passway(double start_num, double end_num, int LC[6][300]);
//int avoid_obstacles(int *X, int *Y, double &angle,double &length);
int avoid_obstacles(int *X, int *Y, int threshold,int work_state,double &angle,double &length,int &start_p,int &end_p);
//int get_effective_point(int *X, int *Y, int EP[3][300], int start_p, int end_p, int&e_point);
int get_effective_point(int *X, int *Y, int EX[300],int EY[300],int start_p, int end_p, int&e_point);
void LineFitting(int x[], int y[], int size, double& theta, double&dist_wall);*/








#endif
