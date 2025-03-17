//********************************************************************************************
// 作   者-->:  杨 东
// 创建时间-->:  2019.08.05
// 修改时间-->:  2019.08.05
// 版   权-->:  重庆邮电大学\自动化学院\汽车电子工程中心\智能汽车技术研究所
//--------------------------------------------------------------------------------------------
// 文档说明 ：
//      该文档的目标将主题代码与ROS框架做隔离的中间层
//      订阅消息的回调函数，实现消息的获取
//**********************************************************************************************

#ifndef ROS_MSG_INTERFACE_H
#define ROS_MSG_INTERFACE_H

#include "ros/ros.h"
#include "v_chassis_interface.h"

#include "custom_msgs/SteeringCmd.h"          //行驶控制消息
#include "custom_msgs/TrqBreCmd.h"
#include "custom_msgs/AEBCmd.h"
#include "custom_msgs/GearCmd.h"

#include "custom_msgs/DoorCmd.h"             //BCM控制消息
#include "custom_msgs/FogLampCmd.h"
#include "custom_msgs/HornsCmd.h"
#include "custom_msgs/LowHeadLightCmd.h"
#include "custom_msgs/SideLampsCmd.h"
#include "custom_msgs/TurnLightCmd.h"
#include "custom_msgs/WindowCmd.h"
#include "custom_msgs/WipersCmd.h"
#include "custom_msgs/EPBCmd.h"
#include "custom_msgs/LKSStatusCmd.h"
#include "custom_msgs/TurnLightStat.h"


#include "custom_msgs/VehicleStat.h"          //其他消息



//行驶控制接口
void get_steering_cmd_callback(const custom_msgs::SteeringCmd::ConstPtr &msg);
void get_trq_bre_cmd_callback(const custom_msgs::TrqBreCmd::ConstPtr &msg);
void get_AEB_cmd_callback(const custom_msgs::AEBCmd::ConstPtr &msg);
void get_Gear_cmd_callback(const custom_msgs::GearCmd::ConstPtr &msg);

//-->>BCM
bool get_Door_cmd_callback( custom_msgs::DoorCmd::Request &Req, custom_msgs::DoorCmd::Response &Res);
bool get_FogLamp_cmd_callback( custom_msgs::FogLampCmd::Request &Req, custom_msgs::FogLampCmd::Response &Res);
bool get_Horns_cmd_callback( custom_msgs::HornsCmd::Request &Req, custom_msgs::HornsCmd::Response &Res);
bool get_Low_Head_Light_cmd_callback( custom_msgs::LowHeadLightCmd::Request &Req, custom_msgs::LowHeadLightCmd::Response &Res);
bool get_SideLamps_cmd_callback( custom_msgs::SideLampsCmd::Request &Req, custom_msgs::SideLampsCmd::Response &Res);
bool get_TurnLight_cmd_callback( custom_msgs::TurnLightCmd::Request &Req, custom_msgs::TurnLightCmd::Response &Res);
bool get_Window_cmd_callback( custom_msgs::WindowCmd::Request &Req, custom_msgs::WindowCmd::Response &Res);
bool get_Wipers_cmd_callback( custom_msgs::WipersCmd::Request &Req, custom_msgs::WipersCmd::Response &Res);
bool get_LKSStatus_cmd_callback( custom_msgs::LKSStatusCmd::Request &Req, custom_msgs::LKSStatusCmd::Response &Res);
bool get_EPB_cmd_callback( custom_msgs::EPBCmd::Request &Req, custom_msgs::EPBCmd::Response &Res);







#endif // ROS_MSG_INTERFACE_H
