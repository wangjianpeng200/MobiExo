//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//*****************************************************************
#ifndef LON_CONTROLLER_H
#define LON_CONTROLLER_H

#include "ros/ros.h"
#include <cstdint>
#include<string>
#include <pqxx/pqxx>
#include<fstream>
#include<filesystem>
#include<sstream>
#include<unordered_map>
#include<unistd.h>
using namespace pqxx;
using namespace std;


/*
//-->>数据结构
enum LON_STATUS{
  RUN    = 0x0,STOP   = 0x1,AEB    = 0x2
};*/
enum LON_STATUS{ FORWARD_ENABLE = 0x0,BACK_ENABLE = 0x3,STOP_ENABLE = 0x1,AEB_ENABLE= 0x2 };
struct VehicleStat{
  float    VehicleSpeed;                   //车速
  float    SAS_SteeringAngle;              //方向盘转角
  float    EMS_EngineSpeed;                //发动机转速
  float    EMS_EngineThrottlePosition;     //发动机节气门位置
  float    EMS_AccPedal;                   //加速踏板位置
  uint8_t  EMS_BrakePedalStatus;           //制动踏板状态
  uint8_t  TCU_GearShiftPositon;           //换挡器位置
};


struct RtkImuStat{
  float    pitch;                          //俯仰角
  float    roll;                           //翻滚角
  float    heading;                        // 航向角
  float    speed2d;                        //车辆速度
};

struct AEB_Cmd {
  uint8_t AEB_enable;
  float   AEB_bre_value;
};

struct Trq_Bre_Cmd{
  uint8_t  bre_enable;
  uint8_t  bre_value;
  uint8_t  trq_enable;
  float    trq_value;
  uint8_t ACC_DecToStop;
  uint8_t ACC_Driveoff;
};

struct AEB {
  struct AEB_Cmd Aeb_Cmd;
  struct Trq_Bre_Cmd Bre_Cmd;
};


struct PID_PARA{
  float last_error;
  float error;
  float KP;
  float KI;
  float KD;
};


struct Request {
    uint8_t request_type;            //请求类型
    float  run_speed;                //行车速度
    float  stop_distance;            //前方停止距离
    float  aeb_distance;             //前方AEB停止距离
};


extern struct PID_PARA pid_para;
extern float speed;

float calculatePID(struct PID_PARA pid);

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


//-->>全局变量
extern struct VehicleStat veh_stat;
extern struct Request request_value;


//-->>三种运行状态处理
/**
 * @brief run_PID_solve
 * @return
 */
struct Trq_Bre_Cmd run_PID_solve();

/**
 * @brief stop_solve
 * @return
 */
struct Trq_Bre_Cmd stop_solve();

/**
 * @brief aeb_solve
 * @return
 */
struct AEB aeb_solve();

struct Trq_Bre_Cmd run_back_solve();

//-->>消息回调外部借口
/**
 * @brief set_vehicle_stat   设置车身状态
 * @param arg
 */
void set_vehicle_stat(struct VehicleStat &arg);

/**
 * @brief set_rtk_imu_stat
 * @param arg
 */
void set_rtk_imu_stat(struct RtkImuStat &arg);

/**
 * @brief set_requre
 * @param arg
 */
void set_requre(struct Request &arg);



#endif //LON_CONTROLLER_H
