//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//      该文档的目标是实现ROS的消息回调，同时将主题代码与ROS框架做隔离的中间层
//*****************************************************************

#ifndef ROS_INTERFACE_LAT_H
#define ROS_INTERFACE_LAT_H

#include "ros/ros.h"

#include "pure_pursuit.h"
#include "custom_msgs/SteeringCmd.h"
#include "custom_msgs/VehicleStat.h"
#include "custom_msgs/Path.h"
#include "geometry_msgs/Pose2D.h"
#include "custom_msgs/TurnLightCmd.h"
#include "custom_msgs/Request.h"

extern uint8_t turn_light_enable;

/**
 * @brief ros_veh_para_init
 */
void ros_veh_para_init();

/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(const custom_msgs::VehicleStat::ConstPtr &msg);
/**
 * @brief ref_path_callback
 * @param msg
 */
void ref_path_callback(const custom_msgs::Path::ConstPtr &msg);

/**
 * @brief veh_pose_callback
 * @param msg
 */
void veh_pose_callback(const geometry_msgs::Pose2D::ConstPtr &msg);

/**
 * @brief turn_light_callback
 * @param msg
 */
void turn_light_callback(const custom_msgs::Request::ConstPtr &msg);

#endif //ROS_INTERFACE_LAT_H
