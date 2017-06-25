#include "tim550.h"

#include <stdio.h>

int main(int argc,char **argv)
{
    char * ip="169.254.85.10";
    char * port="2112";
    int ret=tim550_connect(ip,port);
    if(ret<0)
    {
        printf("connect to sick failed");
        return 0;
    }

    return 0;
}
