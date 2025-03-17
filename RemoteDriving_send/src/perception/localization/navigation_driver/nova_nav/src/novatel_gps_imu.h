#ifndef NOVATEL_GPS_IMU
#define NOVATEL_GPS_IMU

#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>

#define NOVATEL_IMU_PORT 3001

using namespace std;

// Ubuntu 16.04 64bit
// char 1
// int 4
// short 2
// long 8
// float 4
// double 8

typedef unsigned char uchar;
typedef unsigned short ushort;
typedef unsigned int uint;

#pragma pack(1)                 //强制连续排列

//Table 3: Binary Message Header Structure P37
typedef struct long_header_msg                      //长消息头结构体
{
    char sync_1;                        //0xAA
    char sync_2;                        //0x44
    char sync_3;                        //0x12
    uchar header_lgth;                 //消息头长度 28
    ushort message_id;                 //消息ID  126
    char message_type;                 //消息类型，1表示响应消息
    uchar Port_Address;                //端口地址，0表示没有指定端口号，1表示所有虚拟端口号为COM1
    ushort Message_Length;             //消息体长度，不包括循环冗余校验位
    ushort Sequence;                   //通常为0
    uchar Idle_Time;                   //闲置时间
    char Time_Status;                  //时间状态
    ushort Week_Number;                //
    int  ms;                           //
    uint Receiver_Status;              //接收机状态
    ushort Reserved;                   //保留位
    ushort Receiver_S_WVersion;        //接收机软件构建数量
}header_l;

//Table 7: Short Binary Message Header Structure P48
typedef struct short_header_msg                //短消息头结构体
{
    char Synch1;                          //0xAA
    char Synch2;                          //0x44
    char Synch3;                          //0x13
    uchar Message_Length;                //消息长度，不包括消息头和循环冗余校验
    ushort Message_ID;                    //消息ID号
    ushort Week_Number;                     //
    uint Milliseconds;                    //
}header_s;

// BESTPOS 414
typedef struct BEST_POSB_MSG          
{
    header_l msg_header;           //包头
    int sol_stat;                   //INS解算状态 3
    int pos_type;                   //位置类型  56
    double lat;                     //纬度（单位：度）
    double lon;                     //经度（单位：度）
    double hgt;                     //海拔高度（单位：m）
    float undulation;               //波动幅度（单位：m）
    int  datum_id;                  //Datum ID number
    float Lat_vari;                 //纬度标准差
    float Lon_vari;                 //经度标准差
    float hgt_vari;                 //海拔高度标准差
    char stn_id[4];
    float diff_age;
    float sol_age;
    uchar SVs;                      //Solution age in seconds
    uchar solnSVs;                  //Number of satellites tracked
    uchar solnL1SVs;                //Number of satellites used in solution
    uchar solnMultiSVs;
    char Reserved;
    char ext_sol_stat;
    char Galileo_and_BeiDou_sig_mask;
    char GPS_and_GLONASS_sig_mask;
    int  CRC;                        //循环冗余校验
}msg_best_pose;


//INSPVAX消息结构体 P922
typedef struct INSPVAX_MSG
{            
    header_l msg_header;           //包头
    int INS_Status;                 //INS解算状态 3
    int Pos_Type;                   //位置类型  56
    double Lat;                     //纬度（单位：度）
    double Lon;                     //经度（单位：度）
    double Height;                  //海拔高度（单位：m）
    float Undulation;               //波动幅度（单位：m）
    double North_Vel;               //真北方向速度（单位：m/s）
    double East_Vel;                //东向速度（单位：m/s）
    double Up_Vel;                  //天向速度（单位：m/s）
    double Roll;                    //翻滚角（单位：度）
    double Pitch;                   //俯仰角（单位：度）
    double Azimuth;                 //航向角（单位：度）
    float Lat_vari;                 //纬度标准差
    float Lon_vari;                 //经度标准差
    float Height_vari;              //海拔高度标准差
    float North_Vel_vari;           //真北方向速度标准差
    float East_Vel_vari;            //东向速度标准差
    float Up_Vel_vari;              //天向速度标准差
    float Roll_vari;                //翻滚角标准差
    float Pitch_vari;               //俯仰角标准差
    float Azimuth_vari;             //航向角标准差
    int  Ext_sol_stat;              //解算状态的扩展
    ushort  Time_Since_Update;      //位置更新时间或零速修正时间（单位：秒）
    int  CRC;                       //循环冗余校验
}msg_inspvax;

//This log is the short header version of the RAWIMU log (see page 947).  P968
typedef struct RAWIMUS_MSG           //RAWIMUS消息结构体
{
    header_s msg_header;             //包头
    uint Week;                          //
    double Seconds_into_Week;           //
    int IMU_Status;                    //IMU的状态
    int Z_Accel_Output;                //z轴方向速度变化量
    int Y_Accel_Output;                  //y轴方向速度变化量
    int X_Accel_Output;                 //x轴方向速度变化量
    int Z_Gyro_Output;                  //z轴角度变化量
    int Y_Gyro_Output;                   //y轴角度变化量。若为负值，表示沿y轴正方向运动。
    int X_Gyro_Output;                   //x轴角度变化量
    int  CRC;                            //32位循环冗余校验
}msg_rawimus;

//RAWIMU P947
typedef struct RAWIMU_MSG                   //RAWIMUS消息结构体
{
    header_l msg_header;             //包头
    uint Week;                          //
    double Seconds_into_Week;           //
    int IMU_Status;                   //IMU的状态
    int Z_Accel_Output;              //z轴方向速度变化量
    int Y_Accel_Output;                  //y轴方向速度变化量
    int X_Accel_Output;              //x轴方向速度变化量
    int Z_Gyro_Output;               //z轴角度变化量
    int Y_Gyro_Output;                   //y轴角度变化量。若为负值，表示沿y轴正方向运动。
    int X_Gyro_Output;               //x轴角度变化量
    int  CRC;                                  //32位循环冗余校验
}msg_rawimu;

#pragma pack()

int udp_client_init(const int server_port);


#define M_PI 3.14159265358979323846
double rad(double x) ;
double deg(double x) ;

void move_gps_point( double lon1,double lat1,double dist,double heading,double &lon2,double &lat2 );  //移动gps点  

#endif // NOVATEL_GPS_IMU

