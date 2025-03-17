#ifndef SOCKET_API_H
#define SOCKET_API_H

#include <sys/socket.h>

int socket_tcp_client(char *server_ip ,int server_port);

#endif // SOCKET_API_H
