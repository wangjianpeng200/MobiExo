#include "socket_api.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>

int socket_tcp_client(char *server_ip ,int server_port){
    int m_sock;
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(struct sockaddr_in));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(server_port);
    server_addr.sin_addr.s_addr = inet_addr(server_ip);

    m_sock = socket(AF_INET, SOCK_STREAM, 0);
    if(m_sock == -1) {
        printf("create socket fail\n");
        //exit(1);
        return -1;
    }

    /* Connect to caster. */
    while(connect(m_sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_in)) < 0)		//Á¬œÓserver
    {
        printf("connect caster failed\n");
        sleep(1);
    }

    return m_sock;
}
