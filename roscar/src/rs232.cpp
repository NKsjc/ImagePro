#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termio.h>
#include <sys/select.h>
#include <time.h>
#include <math.h>
#include "common_def.h"
static struct termio oterm_attr;
static int baudflag_arr[] = {
    B921600, B460800, B230400, B115200, B57600, B38400,
    B19200, B9600, B4800, B2400, B1800, B1200,
    B600, B300, B150, B110, B75, B50
};
static int speed_arr[] = {
    921600, 460800, 230400, 115200, 57600, 38400,
    19200, 9600, 4800, 2400, 1800, 1200,
    600, 300, 150, 110, 75, 50
};
/******************************************************************************
 * NAME:
 * speed_to_flag
 *
 * DESCRIPTION:
 * Translate baudrate into flag
 *
 * PARAMETERS:
 * speed - The baudrate to convert
 *
 * RETURN:
 * The flag
 ******************************************************************************/
int speed_to_flag(int speed)
{
    int i;

    for (i = 0; i < sizeof(speed_arr)/sizeof(int); i++) {
        if (speed == speed_arr[i]) {
            return baudflag_arr[i];
        }
    }

    fprintf(stderr, "Unsupported baudrate, use 9600 instead!\n");
    return B9600;
}

/******************************************************************************
 * NAME:
 * write_data
 *
 * DESCRIPTION:
 * Write data to serial port
 *
 *
 * PARAMETERS:
 * fd - The fd of serial port to write
 * buf - The buffer to keep data
 * len - The count of data
 *
 * RETURN:
 * Count of data wrote 
 ******************************************************************************/
int write_data(int fd, void *buf, int len)
{
    int count;
    int ret;

    ret = 0;
    count = 0;

    while (len > 0) {

        ret = write(fd, (char*)buf + count, len);
        if (ret < 1) {
            fprintf(stderr, "Write error %s\n", strerror(errno));
            break;
        }
		//usleep(1000);
        count += ret;
        len = len - ret;
    }
	//usleep(3000);
    return count;
}

int read_data(int fd, void *buf, int len,unsigned int timeout){//ms
	int res,recv_len;
	struct timeval Timeout;
	fd_set readfs;
	int maxfd;

	if(timeout ==0){
			timeout = 10;
	}
	Timeout.tv_usec = timeout%1000 *1000; //ms? us? test shows that it's us. 
	Timeout.tv_sec = 0;
	
	FD_ZERO(&readfs);
	FD_SET(fd, &readfs); 
	//FD_SET(fd2, &readfs); //another
	
	maxfd = fd +1;
	res = select(maxfd, &readfs, NULL, NULL, &Timeout);
	if(res <=0 )//timeout
	{
			LDEBUG("serial %d timeout or err :%d \n",fd ,res);
			return -1;
	}else{
		    usleep(10*1000);
		    recv_len = read(fd, buf, len);
		    return recv_len;
	}
}

int read_data_all(int fd, void *buf, int len,unsigned int timeout){//ms
	int res, recv_len, all_data_len = 0;
	struct timeval Timeout;
	fd_set readfs;
	int maxfd;

	if(timeout ==0){
			timeout = 2;
	}
	timeout=10;
	Timeout.tv_usec = timeout * 1000; //us test shows that it's us. 
	Timeout.tv_sec = 0;
	PDEBUG("serial recv %d timeout:%u \n", all_data_len, Timeout.tv_usec);

	//FD_SET(fd2, &readfs); //another
	maxfd = fd +1;
//	while(FD_ISSET(fd,  &readfs)){		
	while(1)
	{		
		ADEBUG("serial recv  timeout:%u \n", Timeout.tv_usec);
		FD_ZERO(&readfs);//每次循环都要清空集合，否则不能检测描述符变化 
		FD_SET(fd, &readfs); //添加描述符 
		res = select(maxfd, &readfs, NULL, NULL, &Timeout);
		if(res <=0 )//timeout
		{
				if(all_data_len == 0)
				{
				    ADEBUG("serial recv %d timeout:%u \n", all_data_len, Timeout.tv_usec);
					ADEBUG("serial %d timeout or err :%d \n",fd ,res);//res=0    0：等待超时，没有可读写或错误的文件 
					return -1;
				}
				else  
				return all_data_len;
		}
		else
		{
				recv_len = read(fd, buf+all_data_len, len);
				all_data_len += recv_len;
				PDEBUG("serial recv %d timeout:%u \n", all_data_len, Timeout.tv_usec);
		}
	}
	return all_data_len;
}
/******************************************************************************
 * NAME:
 * stup_port
 *
 * DESCRIPTION:
 * Set serial port (include baudrate, line control)
 *
 * PARAMETERS:
 * fd - The fd of serial port to setup
 * baud - Baudrate: 9600, ...
 * databits - Databits: 5, 6, 7, 8
 * parity - Parity: 0(None), 1(Odd), 2(Even)
 * stopbits - Stopbits: 1, 2
 *
 * RETURN:
 * 0 for OK; Others for ERROR
 * 
 * Add1 : c_cflag CLOCAL
 * Add2 : c_iflag &= ~(IXON | IXOFF | IXANY);// No software input ctrl. Problem solved! 
 * Add3 :  c_iflag |= IGNBRK
 ******************************************************************************/
int setup_port(int* fd, int baud, int databits, int parity, int stopbits)
{
    struct termio term_attr;

    /* Get current setting */
    if (ioctl(*fd, TCGETA, &term_attr) < 0) {
        return -1;
    }

    /* Backup old setting */
    memcpy(&oterm_attr, &term_attr, sizeof(struct termio));

 //old settings 
	term_attr.c_iflag |= IGNBRK;//unknow
	term_attr.c_iflag |= IGNPAR;//Ignore parity errors
    term_attr.c_iflag &= ~(INLCR | IGNCR | ICRNL | ISTRIP);// no other input processing 
    term_attr.c_iflag &= ~(IXON | IXOFF | IXANY);// No software input ctrl
    /*CREAD: Enable the receiver
     * 通过对 c_cflag 与 c_iflag 的赋值，可以设置波特率、数据位、奇偶校验位、
	 * 	停止位、流控*/
    term_attr.c_oflag &= ~(OPOST | ONLCR | OCRNL); //raw mode (output)
    
    term_attr.c_lflag &= ~(ISIG | ECHO | ICANON | NOFLSH);// 
    
/*//传统模式 CR
   term_attr.c_iflag |= (IGNPAR | ICRNL);  // 0x0d 或0x0a结束符
    //term_attr.c_iflag = 0; //不加结束符
    term_attr.c_oflag =0 ;
//    term_attr.c_oflag &= ~(OPOST | ONLCR | OCRNL); //

    term_attr.c_lflag = ICANON;
    */
    /*仅当open	O_NDELAY参数时生效*/
    term_attr.c_cc[VMIN] = 0;
    term_attr.c_cc[VTIME] = 0;
    
    /*CLOCAL: Do not change the "owner" of port*/
    
     
    term_attr.c_cflag &= ~CBAUD;
    term_attr.c_cflag |= CLOCAL | CREAD | speed_to_flag(baud);
    term_attr.c_cflag &= ~CRTSCTS;//disable hardware handshake
    /* Set databits */
    term_attr.c_cflag &= ~(CSIZE);
    switch (databits) {
        case 5:
            term_attr.c_cflag |= CS5;
            break;

        case 6:
            term_attr.c_cflag |= CS6;
            break;

        case 7:
            term_attr.c_cflag |= CS7;
            break;

        case 8:
        default:
            term_attr.c_cflag |= CS8;
            break;
    }

    /* Set parity */
    switch (parity) {
        case 1: /* Odd parity */
            term_attr.c_cflag |= (PARENB | PARODD);
            break;

        case 2: /* Even parity */
            term_attr.c_cflag |= PARENB;
            term_attr.c_cflag &= ~(PARODD);
            break;

        case 0: /* None parity */
            term_attr.c_cflag &= ~(PARENB);
            break;
            
        default:
            term_attr.c_cflag &= ~(PARENB);
            break;
    }


    /* Set stopbits */
    switch (stopbits) {
        case 2: /* 2 stopbits */
            term_attr.c_cflag |= CSTOPB;
            break;

        case 1: /* 1 stopbits */
        default:
            term_attr.c_cflag &= ~CSTOPB;
            break;
    }

    if (ioctl(*fd, TCSETAW, &term_attr) < 0) {
        return -1;
    }

    if (ioctl(*fd, TCFLSH, 2) < 0) {
        return -1;
    }

    return 0;
}

int reset_port(int fd)
{
    if (ioctl(fd, TCSETAW, &oterm_attr) < 0) {
        return -1;
    }

    return 0;
}

/**
 * 参数：字符串指针 //char serial_port[]=“/dev/ttyS2”
 * 返回文件描述符fd
 * 应用程序:
 * 写 ret = write(fd, (char*)buf, len); //ret返回成功写入的字节数
 *   ？ fputs 可以吗？
 * 读 NULL == fgets(buff_std, sizeof(buff_std), reader->port)
 */
int serial_init(const char *device)
{
	int fd;

//    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY, 0);//immediately return after call "read"
    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);		//block or wait as c_cc set
	if (fd < 0) {
        fprintf(stderr, "open <%s> error %s\n", device, strerror(errno));
        return -1;
    }

    if (setup_port(&fd, BIT_RATE, 8, 0, 1)) {
        fprintf(stderr, "setup_port error %s\n", strerror(errno));
        close(fd);
        return -1;
    }

	return fd;
}
