//********************************************************************************************
//新小底盘车（pix普朗克新版）
//**********************************************************************************************

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <cstring>
#include <cstdint>

//-------------------------------------------------------------控制报文-------------------------------------------------------------------------------

//转向、制动控制报文
struct MSG_ID301 {
    uint8_t ACU_SteerEn;  //车辆转向控制使能 "0:disable 1:enable" 
    uint8_t ACU_BrakeEn;  //车辆制动控制使能 "0:disable 1:enable"
    uint8_t ACU_BrakeUrgentEn;  //车辆紧急制动控制使能 "0:disable 1:enable" 
    uint8_t ACU_ParkEn;   //停车使能开关 "0:disable 1:enable"
    uint8_t ACU_Brake_Conway;  //制动指令控制方式 "0:制动踏板开度控制 1:制动减速度控制"
    float   ACU_Steer_angel;   //车辆转向控制 -580 - +580  精度0.1
    uint8_t ACU_Steer_sp;   //转向角速度   0-540  精度：10度/秒
    uint8_t ACU_Brake_disP; //控制制动踏板开度  0-100  精度1
    float   ACU_Brake_decA; //控制制动减速度    0-10   精度0.1
    uint8_t ACU_life;  
};

//驱动控制报文
struct MSG_ID302 {
    uint8_t ACU_M_Drive_En;//驱动使能信号 
    uint8_t ACU_M_Gear_En;//档位使能信号 
    uint8_t ACU_M_Gears;//档位设置“0:P档 / 1:R档 / 2:N档 / 3:D档”
    uint8_t ACU_M_Conway;//驱动控制方式  默认只有0
    uint8_t ACU_M_Drive_value;  //车辆目标加速踏板开度值  0-100
    uint8_t ACU_M_Drive_trq;  //车辆车轮目标转矩值  0-45  精度1nm
    float   ACU_M_Drive_acc;  //车辆目标加速度值  0-10 精度0.1
    double  ACU_M_Drive_sp;   //车辆速度控制  
};

//灯光、喇叭控制报文
struct MSG_ID303 {
    uint8_t ACU_AllLightEn;  //灯光总使能
    uint8_t ACU_TurnLightEn;  //转向灯使能
    uint8_t ACU_BrakeLightEn;  //制动灯使能

    uint8_t ACU_VB_Horn_con;  //喇叭使能
};


bool frame_encapsulation_ID301(const struct MSG_ID301& msg, struct can_frame& frame);
bool frame_encapsulation_ID302(const struct MSG_ID302& msg, struct can_frame& frame);
bool frame_encapsulation_ID303(const struct MSG_ID303& msg, struct can_frame& frame);


//--------------------------------------状态报文----------------------------------------------------

//车辆转向与制动状态反馈
struct MSG_ID3f1 {
    uint8_t VCU_Switch_sta;     //手动/自动模式切换
    float VCU_SteerAngel_sta;   //转向角度反馈 
    uint8_t VCU_BrakeValue_sta; //刹车踏板反馈值
    uint8_t VCU_BraEn_sta;      //制动使能状态 "0:disable / 1:enable"
};

//车辆纵向消息
struct MSG_ID3f2 {
    uint8_t VCU_Gear_sta;       //车辆真实档位
    uint8_t VCU_DriveValue_sta; //油门踏板反馈值
    float VCU_Speed_sta;        //速度反馈值
};

//车辆灯光、喇叭消息
struct MSG_ID3f3 {
    uint8_t VCU_LowHighLight_sta; //近、远光灯使能
    uint8_t VCU_TurnLight_sta;    //转向灯使能
    uint8_t VCU_BrakeLight_sta;   //制动灯使能
    uint8_t VCU_VB_Horn_sta;      //喇叭使能
};


void frame_parsing_ID3f1(const unsigned char* frame_data, struct MSG_ID3f1& msg);
void frame_parsing_ID3f2(const unsigned char* frame_data, struct MSG_ID3f2& msg);
void frame_parsing_ID3f3(const unsigned char* frame_data, struct MSG_ID3f3& msg);




enum CanFrameID_1{
  ID3f1 = 0x3f1, ID3f2 = 0x3f2, ID3f3 = 0x3f3        //->执行器控制报文ID
};

#endif // CAN_PROTOCOL_H
