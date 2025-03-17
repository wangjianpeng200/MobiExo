
#include "can_protocol.h"
#include <iostream>
//#include "v_ros_msg_interface.h"
using namespace std;

/********************************************控制报文封装函数 begin**********************************************/

//转向、制动控制报文封装
bool frame_encapsulation_ID301(const struct MSG_ID301& msg, struct can_frame& frame){
    memset(&frame,0,sizeof (frame));                             //清空消息缓存区
    frame.can_id    =   0x301;                                      //设置消息ID
    frame.can_dlc   =   8;                                          //设置数据字节数

    frame.data[0] = static_cast<uint8_t>((msg.ACU_SteerEn & 0x01) << 2) + static_cast<uint8_t>((msg.ACU_BrakeEn & 0x01) << 3) +
                    static_cast<uint8_t>((msg.ACU_BrakeUrgentEn & 0x01) << 4) + static_cast<uint8_t>((msg.ACU_Brake_Conway & 0x01) << 6);
    frame.data[1] = static_cast<uint8_t>((int)((msg.ACU_Steer_angel + 1575) * 10) & 0x00ff);
    frame.data[2] = static_cast<uint8_t>(((int)((msg.ACU_Steer_angel + 1575) * 10)  & 0xff00) >> 8);
    frame.data[3] = static_cast<uint8_t>((int)(msg.ACU_Steer_sp * 0.1) & 0xff);
    frame.data[4] = static_cast<uint8_t>(msg.ACU_Brake_disP & 0xff);
    frame.data[5] = static_cast<uint8_t>((int)(msg.ACU_Brake_decA * 10)& 0xff);
    frame.data[6] = 0x00;
    frame.data[7] = static_cast<uint8_t>(msg.ACU_life & 0x00);
    return true;
}

//驱动控制报文封装
bool frame_encapsulation_ID302(const struct MSG_ID302& msg, struct can_frame& frame){
    memset(&frame,0x00,sizeof (frame));                             //清空消息缓存区
    frame.can_id    =   0x302;                                      //设置消息ID
    frame.can_dlc   =   8;                                          //设置数据字节数

    frame.data[0] = static_cast<uint8_t>(msg.ACU_M_Drive_En & 0x01) + static_cast<uint8_t>((msg.ACU_M_Gear_En & 0x01) << 1)+ 
                    static_cast<uint8_t>((msg.ACU_M_Gears & 0x03) << 4) + static_cast<uint8_t>((msg.ACU_M_Conway & 0x03) << 6);
    frame.data[1] = static_cast<uint8_t>(msg.ACU_M_Drive_value & 0xff);
    frame.data[2] = static_cast<uint8_t>(msg.ACU_M_Drive_trq & 0xff);
    frame.data[3] = static_cast<uint8_t>((int)(msg.ACU_M_Drive_acc *10) & 0xff);
    frame.data[4] = static_cast<uint8_t>((int)(msg.ACU_M_Drive_sp * 256) & 0x00ff);
    frame.data[5] = static_cast<uint8_t>(((int)(msg.ACU_M_Drive_sp * 256) & 0xff00) >> 8);
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return true;
}

//灯光、喇叭控制报文封装
bool frame_encapsulation_ID303(const struct MSG_ID303& msg, struct can_frame& frame){
    memset(&frame,0x00,sizeof (frame));                             //清空消息缓存区
    frame.can_id    =   0x303;                                      //设置消息ID
    frame.can_dlc   =   8;                                          //设置数据字节数

    frame.data[0] = static_cast<uint8_t>(msg.ACU_AllLightEn & 0x03) + static_cast<uint8_t>((msg.ACU_TurnLightEn & 0x03) << 2)+ 
                    static_cast<uint8_t>((msg.ACU_BrakeLightEn & 0x01) << 7);
    frame.data[1] = static_cast<uint8_t>((int)(msg.ACU_VB_Horn_con) & 0x01);
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    return true;
}

/********************************************状态报文解析函数 begin**********************************************/

//车辆转向、制动状态反馈解析
void frame_parsing_ID3f1(const unsigned char* frame_data, struct MSG_ID3f1& msg){
    msg.VCU_Switch_sta     = (frame_data[0] & 0x08) >> 3;
    msg.VCU_SteerAngel_sta = (static_cast<uint16_t>(frame_data[2] << 8) + static_cast<uint16_t>(frame_data[1]))/10 - 1575.0;
    msg.VCU_BrakeValue_sta = (frame_data[3] & 0xff);
    msg.VCU_BraEn_sta      = (frame_data[4] & 0x02) >> 1;

}

//车辆纵向消息反馈
void frame_parsing_ID3f2(const unsigned char* frame_data, struct MSG_ID3f2& msg){
    msg.VCU_Gear_sta       = (frame_data[0] & 0x03);
    msg.VCU_DriveValue_sta = (frame_data[1] & 0xff);
    msg.VCU_Speed_sta      = (static_cast<uint16_t>(frame_data[3] << 8) + static_cast<uint16_t>(frame_data[2]))/256;

}


//车辆灯光反馈
void frame_parsing_ID3f3(const unsigned char* frame_data, struct MSG_ID3f3& msg){
    msg.VCU_LowHighLight_sta = (frame_data[0] & 0x03);
    msg.VCU_TurnLight_sta    = (frame_data[0] & 0x0C) >> 2;
    msg.VCU_BrakeLight_sta   = (frame_data[0] & 0x80) >> 8;
    msg.VCU_VB_Horn_sta      = (frame_data[2] & 0x01);
}





