#include "v_ros_msg_interface.h"


///-----------------------------------------------------------------------------------------------------
///-----------------------------------------------------------------------------------------------------
///-----------------------------------------------------------------------------------------------------
//-->>行驶控制接口
/**
 * @brief get_steering_cmd_callback
 * @param msg
 */
void get_steering_cmd_callback(const custom_msgs::SteeringCmd::ConstPtr &msg){
    set_steering_cmd(msg->SteeringAngle);
    return ;
}

/**
 * @brief get_trq_bre_cmd_callback
 * @param msg
 */
void get_trq_bre_cmd_callback(const custom_msgs::TrqBreCmd::ConstPtr &msg){
    set_trq_bre_cmd(msg->trq_enable,msg->trq_value_3,msg->bre_enable,msg->bre_value,msg->ACC_DecToStop,msg->ACC_Driveoff);
    return ;
}


/**
 * @brief get_AEB_cmd_callback
 * @param msg
 */
void get_AEB_cmd_callback(const custom_msgs::AEBCmd::ConstPtr &msg){
    set_aeb_cmd(msg->AEB_enable,msg->AEB_bre_value);
    return ;
}

/**
 * @brief get_Gear_cmd_callback
 * @param msg
 */
void get_Gear_cmd_callback(const custom_msgs::GearCmd::ConstPtr &msg){
     set_Gear_cmd(msg->APA_GearEnable,msg->APA_GearRequest);
  return ;
}

