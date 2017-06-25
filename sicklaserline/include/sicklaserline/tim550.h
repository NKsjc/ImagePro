#ifndef _NAV350_H
#define _NAV350_H
#define PI 3.14159
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
  
#endif
