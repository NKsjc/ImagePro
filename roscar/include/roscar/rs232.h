#ifndef _RS485_H
#define _RS485_H

int serial_init(const char *device);
int write_data(int fd, void *buf, int len);
int read_data(int fd, void *buf, int len,unsigned int timeout);
int read_data_all(int fd, void *buf, int len,unsigned int timeout);

#endif
