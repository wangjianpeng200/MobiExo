#include "ros_interface_lon.h"
#define PI 3.1415926

/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(const custom_msgs::VehicleStat::ConstPtr &msg){
    struct VehicleStat temp;
    temp.VehicleSpeed                = msg->veh_speed;
    temp.SAS_SteeringAngle           = msg->SAS_SteeringAngle;
    temp.EMS_EngineSpeed             = msg->EMS_EngineSpeed;
    temp.EMS_EngineThrottlePosition  = msg->EMS_EngineThrottlePosition;
    temp.EMS_AccPedal                = msg->EMS_AccPedal;
    temp.EMS_BrakePedalStatus        = msg->EMS_BrakePedalStatus;
    temp.TCU_GearShiftPositon        = msg->TCU_GearShiftPositon;
    set_vehicle_stat(temp);
    return;
}

/**
 * @brief veh_pose_callback
 * @param msg
 */
void veh_pose_callback(const custom_msgs::NaviData::ConstPtr &msg){
    struct RtkImuStat temp;
    temp.pitch   = msg->pitch;          //俯仰角
    temp.roll    = msg->roll;           //翻滚角
    temp.heading = msg->heading;        // 航向角
    temp.speed2d = msg->speed2d;        //车辆速度
    set_rtk_imu_stat(temp);
    uint8_t i = 0;
    return;
}


/**
 * @brief run_req_callback
 * @param msg
 */
void run_req_callback(const custom_msgs::Request::ConstPtr &msg)
{
    struct Request temp;
    temp.request_type  = msg->reques_type;
    temp.run_speed     = msg->run_speed;
    temp.aeb_distance  = msg->aeb_distance;
    temp.stop_distance = msg->stop_distance;
    set_requre(temp);
    return;
}

