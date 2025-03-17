#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dtu_api.h"
#include <unistd.h>

char index2chr(int index){
    const char base64_code_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    return base64_code_table[index];
}

//用户名:密码转化为base64,网页版也可以实现
int base64_encode(char *src, char *result){
    char temp[3] = {0};
    int i = 0, j = 0, count = 0;
    int len = strlen(src);
    if(len==0)
        return -1;
    if(len%3 != 0){
        count = 3 - len%3;
    }
    while(i < len){
        strncpy(temp, src+i, 3);
        result[j+0] = index2chr((temp[0]&0xFC)>>2);
        result[j+1] = index2chr(((temp[0]&0x3)<<4) | ((temp[1]&0xF0)>>4));
        if(temp[1] == 0)
            break;
        result[j+2] = index2chr(((temp[1]&0xF)<<2) | ((temp[2]&0xC0)>>6));
        if(temp[2] == 0)
            break;
        result[j+3] = index2chr(temp[2]&0x3F);
        i+=3;
        j+=4;
        memset(temp, 0x0, 3);
    }

    while(count){
        result[j+4-count] = '=';
        --count;
    }

    return 0;
}

//获取当前服务器的挂载点列表
void ntrip_caster_source_table(struct NtripConf &conf){
    int m_sock = socket_tcp_client(conf.ip,conf.port);
    char recv_buf[20480] = {0};
    char str1[4096];
    printf("--------------------NtripCaster返回源列表---------------------\n");
    set_ntrip_conf(conf,str1,4096);
    int ret = send(m_sock, str1, strlen(str1), 0);
    if(ret < 0){
        printf("send request fail\n");
        exit(1);
    }
    usleep(1000*3000);
    ret = recv(m_sock, (void *)recv_buf, sizeof(recv_buf), 0);
    printf("%s",recv_buf);
}

//设置HTTP请求消息
void set_ntrip_conf(struct NtripConf &conf,char *msg_request,u_int32_t len){
    memset(msg_request, 0, len);
    char GET[64] = {0};
    sprintf(GET,"GET /%s HTTP/1.0\r\n",conf.mountpoint);
    char *User_Agent = (char *) "User-Agent: NTRIP GNSSInternetRadio/1.4.10\r\n";
    char *Accept     = (char *) "Accept: */*\r\n";
    char *Connection = (char *) "Connection: close\r\n";
    char Authorization[64] = {0};
    char usr_key_info[64] = {0};
    char usr_key_base64[64] = {0};
    sprintf(usr_key_info,"%s:%s",conf.usr_id,conf.pass_wd);
    base64_encode(usr_key_info, usr_key_base64);                       //转成Base64编码
    sprintf(Authorization,"Authorization: Basic %s\r\n",usr_key_base64);
    sprintf(msg_request,"%s%s%s%s%s\r\n",GET,User_Agent,Accept,Connection,Authorization);

    printf("%s",msg_request);
    return ;
}

//校验码
int check_sum(char *src){
    int sum = 0, num = 0;
    sscanf(src, "%*[^*]*%x", &num);
    for(int i = 1; src[i] != '*'; ++i){
        sum ^= src[i];
    }
    return sum - num;
}

