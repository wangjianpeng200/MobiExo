#ifndef SERIAL_PORT_INTERFACE
#define SERIAL_PORT_INTERFACE


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>

int uart_open(const char *ttysn);

int uart_conf_set(int fd,int nBaud,int nBits,int nStop,char nEvent);

#endif // SERIAL_PORT_INTERFACE

