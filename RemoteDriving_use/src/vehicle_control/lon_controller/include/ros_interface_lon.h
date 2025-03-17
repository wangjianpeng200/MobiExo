//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//*****************************************************************
#ifndef ROS_INTERFACE_LON_H
#define ROS_INTERFACE_LON_H


#include "lon_controller.h"
//车身状态
#include "custom_msgs/AEBCmd.h"
#include "custom_msgs/VehicleStat.h"
#include "custom_msgs/NaviData.h"

//停车、AEB、正常行驶
#include "custom_msgs/Request.h"

//-->>控制消息发布
#include "custom_msgs/TrqBreCmd.h"
#include "custom_msgs/GearCmd.h"

//毫米波雷达检测值
#include "custom_msgs/RadarRawObjectArray.h"
#include "custom_msgs/RadarRawObject.h"

//相机检测值
#include "custom_msgs/Lane_Stat.h"




//-->>消息回调
/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(const custom_msgs::VehicleStat::ConstPtr &msg);

/**
 * @brief veh_pose_callback
 * @param msg
 */
void veh_pose_callback(const custom_msgs::NaviData::ConstPtr &msg);


/**
 * @brief WaveRadar_callback
 * 在直线道路上前方有无车辆出现
 * @param msg
 */
void WaveRadar_callback(const custom_msgs::RadarRawObjectArray::ConstPtr &msg);
/**
 * @brief camareLane_callback
 * 相机检测车道曲率
 * @param msg
 */

void camareLane_callback(const custom_msgs::Lane_Stat::ConstPtr &msg);

/**
 * @brief run_req_callback
 * @param msg
 */
void run_req_callback(const custom_msgs::Request::ConstPtr &msg);

#endif //ROS_INTERFACE_LON_H
