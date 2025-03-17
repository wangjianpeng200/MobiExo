#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "dtu_api.h"
#include "uart_api.h"
#include <unistd.h>
#include <string>

#include <iostream>
#include <iomanip>

#include <ros/ros.h>

#include <custom_msgs/NaviData.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dtu_node");
    ros::NodeHandle nh;

//    Ntrip Client获取源列表
//    struct NtripConf conf_source = {"60.205.8.49",8002,"","",""};
//    ntrip_caster_source_table(conf_source);
    char gpgga_buff[1024] = {0};
    char gpgga[] = "$GPGGA,024305.00,2932.26580425,N,10636.11411454,E,1,20,0.7,424.145,M,-29.551,M,,*4F\r\n";

    std::string uart_num;
    nh.param<std::string>("uart_num", uart_num,"/dev/ttyUSB0");
    int uart = uart_open(uart_num.c_str());
    fcntl(uart,F_SETFL,FNDELAY);//非阻塞read
    uart_conf_set(uart,115200,8,1,'N');

    //struct NtripConf conf = {"60.205.8.49",8002,"qxtjbl001","2c842f3","RTCM32_GGB"};
    //struct NtripConf conf = {"60.205.8.49",8002,"qxtjbl002","c9b3781","RTCM32_GGB"};
    struct NtripConf conf = {"60.205.8.49",8002,"qxtkqb003","e62c127","RTCM32_GGB"};
    // struct NtripConf conf = {"60.205.8.49",8002,"qxtjbl003","343ae0a","RTCM32_GGB"};
    //struct NtripConf conf = {"60.205.8.49",8002,"qxnmhm001","f33dc3e","RTCM32_GGB"};
    //struct NtripConf conf = {"60.205.8.49",8002,"mcqkms004","e80360f","RTCM32_GGB"};
    //struct NtripConf conf = {"60.205.8.49",8002,"qxnmhm002","1bd8177","RTCM32_GGB"};
    //struct NtripConf conf = {"60.205.8.49",8002,"qxtkqb001","a5efa77","RTCM32_GGB"};
    
    while(ros::ok()){
        //登录初始化
        int m_sock = socket_tcp_client("60.205.8.49",8002);
        fcntl(m_sock,F_SETFL,FNDELAY);
        int ret_wr;
        int ret_send;
        int ret;
        char recv_buf[1024] = {0};
        char msg_request_rtk[4096];
        set_ntrip_conf(conf,msg_request_rtk,sizeof (msg_request_rtk));
        memset(recv_buf, 0, 1024);
        while(send(m_sock, msg_request_rtk, strlen(msg_request_rtk), 0) == -1)
        {
            printf("send request fail\n");
            sleep(1);
        }
        printf("send request OK\n");
        while (ros::ok())
        {
            ret = recv(m_sock, (void *)recv_buf, sizeof(recv_buf), 0);;
            if(ret > 0 && !strncmp(recv_buf, "ICY 200 OK\r\n", 12)){
                printf("ICY 200 OK\n");
                break;
            }
        }

        while(ros::ok())
        {   //从惯导获取gpgga发送给千寻服务器
            memset(gpgga_buff,0,1024);
            //send(m_sock, gpgga,sizeof (gpgga), 0);
            if((ret = read(uart,gpgga_buff,1024)) > 0 ){                 //read com data gpgga from mobile 
                //cout<<"read ok :"<< ret << endl;
                if((ret_send = send(m_sock, gpgga_buff,ret, 0)) > 0)    //seed com data gpgga to caster
                {
                    //cout<<"send ok :"<< ret_send << endl;
                }else{
                    cout<<"send gpgga failed !"<< endl;
                    break;
                }
            } 

            //接收RTCM报文，写入惯导
            memset(recv_buf,0,1024);
            if((ret = recv(m_sock, (void *)recv_buf, sizeof(recv_buf), 0)) >0){   //read RTCM32 TCP from caster
                //cout<<"recv ok  :"<< ret << endl;                    
                if((ret_wr = write(uart,recv_buf,ret)) >0)                         // write RTCM32 to mobile
                     //cout<<"write ok :"<< ret_wr << endl;
                     ;
            }else if(ret == 0){
        	    cout<<"no caster data recv!!!!"<< endl;
                break;	
            }
        }
        close(m_sock);
    }
    
    return  0;
}

