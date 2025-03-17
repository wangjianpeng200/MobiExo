//********************************************************************************************
// 作   者-->:  杨 东
// 创建时间-->:  2019.08.05
// 修改时间-->:  2019.08.05
// 版   权-->:  重庆邮电大学\自动化学院\汽车电子工程中心\智能汽车技术研究所
//--------------------------------------------------------------------------------------------
// 文档说明 ：
//**********************************************************************************************

#ifndef V_CHASSIS_INTERFACE_H
#define V_CHASSIS_INTERFACE_H

#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <can_protocol.h>
#include "socketcan.h"
#include <string>
#include "ros/ros.h"
#include "custom_msgs/SteeringCmd.h"          //行驶控制消息



#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
using namespace std;
#define NO_LOCAL_DEBUG

struct VehicleStat{
  float    VehicleSpeed;                //车速
  float    SAS_SteeringAngle;           //方向盘转角
  uint8_t  VcuBrkPedlSts;              //减速踏板状态
  uint8_t  TCU_GearShiftPositon;        //换挡器位置
};

struct TurnLight_status{
  uint8_t Veh_LowHighLight;
  uint8_t Veh_TurnLightLe;       //->>转向灯  0x0:TurnOff; 0x1:TurnL
  uint8_t Veh_BrakeLight;
  uint8_t Veh_VU_Horn;
};

extern int can_fd;
extern float kp,ki,kd;

/**
 * @brief start_status_monitor_thread 开启监控线程
 * @return
 */
pthread_t start_status_monitor_thread();

pthread_t start_manual_auto_driving_thread();


/**
 * @brief start_send_timer  开启命令发送定时器
 * @param t_ms 定时器设定时间
 */
void start_send_timer(int t_ms);

void get_veh_status(struct VehicleStat &stutus);
void get_TurnLight_status(struct TurnLight_status &status);

float calculatePID(const float KP,const float KI,const float KD,float angel1);
void vehicle_clear();
void vehicle_ready();
void Relase_EPB();
void Request_EPB();

//------------------------------------------------->>消息回调接口------------------------------------------------
//-->>CTRL 消息回调函数
void set_steering_cmd(const float angle);
void set_trq_bre_cmd(uint8_t trq_enable,const float trq_value,const uint8_t bre_enable,const float bre_value,const uint8_t DecToStop,const uint8_t Driveoff);
void set_aeb_cmd(const uint8_t aeb_enable,const float aeb_bre_value);
void set_Gear_cmd(const uint8_t Enable,uint8_t Request);

//---------------------------------------------------------------------------------------------------------------

#endif // V_CHASSIS_INTERFACE_H
