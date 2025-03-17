#include "ros_interface_lat.h"

uint8_t turn_light_enable = false;

/**
 * @brief ros_veh_para_init
 */
void ros_veh_para_init()
{
  struct VehPara temp;
  ros::param::get("~veh_len",temp.len);
  ros::param::get("~veh_width",temp.width);
  ros::param::get("~veh_hight",temp.hight);
  ros::param::get("~veh_mass",temp.mass);
  ros::param::get("~veh_f_tread",temp.f_tread);
  ros::param::get("~veh_r_tread",temp.r_tread);
  ros::param::get("~veh_wheelbase",temp.wheelbase);
  ros::param::get("~veh_steering_ratio",temp.steering_ratio);
  ros::param::get("~veh_max_wheel_turn_angle",temp.max_wheel_angle);
  ros::param::get("~veh_max_steer_turn_angle",temp.max_steer_angle);
  ros::param::get("~veh_wheel_diam",temp.wheel_diam);

  std::cout <<"----------------------------------"<<std::endl;
  std::cout <<"-        3号智能车车辆参数       -"<<std::endl;
  std::cout <<"----------------------------------"<<std::endl;
  std::cout<<"  len             : "<<temp.len<<std::endl;
  std::cout<<"  width           : "<<temp.width<<std::endl;
  std::cout<<"  hight           : "<<temp.hight<<std::endl;
  std::cout<<"  mass            : "<<temp.mass<<std::endl;
  std::cout<<"  f_tread         : "<<temp.f_tread<<std::endl;
  std::cout<<"  r_tread         : "<<temp.r_tread<<std::endl;
  std::cout<<"  wheelbase       : "<<temp.wheelbase<<std::endl;
  std::cout<<"  wheel_diam      : "<<temp.wheel_diam<<std::endl;
  std::cout<<"  steering_ratio  : "<<temp.steering_ratio<<std::endl;
  std::cout<<"  max_steer_angle : "<<temp.max_steer_angle<<std::endl;
  std::cout<<"  max_wheel_angle : "<<temp.max_wheel_angle<<std::endl;
  std::cout <<"----------------------------------"<<std::endl;

  pure_veh_para_init(temp);      //纯跟踪算法获取车辆参数
  return;
}

/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(const custom_msgs::VehicleStat::ConstPtr &msg){
    pure_set_veh_speed(msg->veh_speed);
    return ;
}

/**
 * @brief ref_path_callback
 * @param msg
 */
void ref_path_callback(const custom_msgs::Path::ConstPtr &msg){
   pure_set_ref_path(msg->x_ref,msg->y_ref);
   return ;
}


// void turn_light_callback(const custom_msgs::Request::ConstPtr &msg){

//     turn_light_enable = msg->is_converge;
//     return;
// }

