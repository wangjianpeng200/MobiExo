#ifndef NOVATEL_GPS_IMU
#define NOVATEL_GPS_IMU

#include <stdint.h>
#pragma pack(1)                 //强制连续排列

//Table 3: Binary Message Header Structure P37
struct msg_header{                      //长消息头结构体
    int8_t   sync_1;                        //-->1   0xAA
    int8_t   sync_2;                        //-->2   0x44
    int8_t   sync_3;                        //-->3   0x12
    uint8_t  header_lgth;                   //-->4   消息头长度 28 1C
    uint16_t message_id;                    //-->5   消息ID 42 268 1465
    int8_t   message_type;                  //-->6   消息类型，1表示响应消息
    uint8_t  Port_Address;                  //-->7   端口地址，0表示没有指定端口号，1表示所有虚拟端口号为COM1
    uint16_t Message_Length;                //-->8   消息体长度，不包括循环冗余校验位
    uint16_t Sequence;                      //-->9   通常为0
    uint8_t  Idle_Time;                     //-->10  闲置时间
    uint8_t  Time_Status;                   //-->11  时间状态
    uint16_t Week_Number;                   //-->12
    uint32_t ms;                            //-->13
    uint32_t Receiver_Status;               //-->14  接收机状态
    uint16_t Reserved;                      //-->15  保留位
    uint16_t Receiver_S_WVersion;           //-->16  接收机软件构建数量
};

// BESTPOS消息结构体42 P414
struct BEST_POSB_MSG{
    msg_header header;           //包头
    uint32_t sol_stat;                   //INS解算状态 3
    uint32_t pos_type;                   //位置类型  56
    double lat;                     //纬度（单位：度）
    double lon;                     //经度（单位：度）
    double hgt;                     //海拔高度（单位：m）
    float undulation;               //波动幅度（单位：m）
    uint32_t  datum_id;                  //Datum ID number
    float Lat_vari;                 //纬度标准差
    float Lon_vari;                 //经度标准差
    float hgt_vari;                 //海拔高度标准差
    int8_t stn_id[4];
    float diff_age;
    float sol_age;
    uint8_t SVs;                      //Solution age in seconds
    uint8_t solnSVs;                  //Number of satellites tracked
    uint8_t solnL1SVs;                //Number of satellites used in solution
    uint8_t solnMultiSVs;
    int8_t Reserved;
    int8_t ext_sol_stat;
    int8_t Galileo_and_BeiDou_sig_mask;
    int8_t GPS_and_GLONASS_sig_mask;
    int32_t  CRC;                        //循环冗余校验
};


//INSPVAX消息结构体1465 P922
struct INSPVAX_MSG{
    msg_header header;           //包头
    int32_t INS_Status;                 //INS解算状态 3
    int32_t Pos_Type;                   //位置类型  56
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
    int32_t  Ext_sol_stat;              //解算状态的扩展
    int16_t  Time_Since_Update;      //位置更新时间或零速修正时间（单位：秒）
    int32_t  CRC;                       //循环冗余校验
};

//RAWIMU消息结构体268 P947
struct RAWIMU_MSG{                   //RAWIMUS消息结构体

    msg_header header;                         //包头
    uint32_t Week;                             //
    double   Seconds_uint16_to_Week;           //
    int32_t  IMU_Status;                       //IMU的状态
    int32_t  Z_Accel_Output;                   //z轴方向速度变化量
    int32_t  Y_Accel_Output;                   //y轴方向速度变化量
    int32_t  X_Accel_Output;                   //x轴方向速度变化量
    int32_t  Z_Gyro_Output;                    //z轴角度变化量
    int32_t  Y_Gyro_Output;                    //y轴角度变化量。若为负值，表示沿y轴正方向运动。
    int32_t  X_Gyro_Output;                    //x轴角度变化量
    int32_t  CRC;                              //32位循环冗余校验
};

#pragma pack()
#endif // NOVATEL_GPS_IMU

