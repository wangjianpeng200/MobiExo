#include "lon_controller.h"

//-->>全局变量
struct VehicleStat veh_stat      = {0,0,0,0,0,0,0};
struct Request     request_value = {LON_STATUS::FORWARD_ENABLE,0,0,0};

//-->>静态全局变量
static struct RtkImuStat rtk_stat = {0,0,0,0};


void set_vehicle_stat(struct VehicleStat &arg){
    veh_stat = arg;
    return ;
}
void set_rtk_imu_stat(struct RtkImuStat &arg){
    rtk_stat = arg;
    return ;
}
void set_requre(struct Request &arg){
    request_value = arg;
    return ;
}




//--------------------------------------------------------
/**
 * @brief stop_solve
 * @return
 */
struct Trq_Bre_Cmd stop_solve(){
  struct Trq_Bre_Cmd temp = {0,0,0,0,0,0};
  temp.trq_enable = 0;
  temp.bre_enable = 1;
  temp.bre_value  = 2;
  return temp;
}
 





/**
 * @brief aeb_solve
 * @return
 */
struct AEB aeb_solve(){

    struct AEB_Cmd AEB_temp = {0,0};
    struct Trq_Bre_Cmd bre_temp = {0,0,0,0,0,0};
    int dcc_temp = 0;
    AEB_temp.AEB_enable = 0;
    bre_temp.trq_enable = 0;
    bre_temp.trq_value  = 0;
    bre_temp.bre_enable = 1;

    bre_temp.bre_value =request_value.aeb_distance;
    AEB_temp.AEB_bre_value=request_value.aeb_distance;
    if(request_value.aeb_distance<0) {
      AEB_temp.AEB_enable = 1;
    }
    AEB_temp.AEB_bre_value = AEB_temp.AEB_bre_value < -9.9 ? -9.9 : AEB_temp.AEB_bre_value;
    struct AEB Aeb = {AEB_temp,bre_temp};
    return Aeb;
}



float calculatePID(struct PID_PARA pid){
  float proportion = pid.KP * pid.error;
  float integration = pid.KI * (pid.error + pid.last_error);
  float differential = pid.KD * (pid.error - 2 * pid.last_error + integration);
  float increment = proportion + integration + differential;
  if (increment > 90)  increment = 90;
  else if (increment < 0) increment = 0;
  pid.last_error = pid.error;
  return increment;
}



/**
 * @brief run_PID_solve
 * @return
 */
struct Trq_Bre_Cmd run_PID_solve(){                     
  static struct Trq_Bre_Cmd cur_value = {0,0,0,0,0,0};
  static float ref_v = speed; 
  //-->>人员干预制动处理
  if(veh_stat.EMS_BrakePedalStatus == 1){
    cur_value.bre_value   = 0;
    cur_value.bre_enable  = 0;
    cur_value.trq_enable  = 0;
    cur_value.trq_value   = 0;
    return cur_value;
  }
  cur_value.trq_enable = 1;
  pid_para.error = ref_v - veh_stat.VehicleSpeed;
  // ROS_WARN_THROTTLE(1, "flag = %d", cur_value.trq_enable);
  cur_value.trq_value = (int)(calculatePID(pid_para) * 1.11);
  ROS_WARN_THROTTLE(1, "VehicleSpeed = %f", cur_value.trq_value);
  return cur_value;

}



struct Trq_Bre_Cmd run_back_solve(){
  static struct Trq_Bre_Cmd cur_value = {0,0,0,0,0,0};
  float ref_v = 7;        //上位机速度
  ROS_INFO_THROTTLE(1, "VehicleSpeed = %f", veh_stat.VehicleSpeed);

}





