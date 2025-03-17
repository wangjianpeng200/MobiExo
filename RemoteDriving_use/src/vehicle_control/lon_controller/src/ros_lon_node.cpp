#include "ros_interface_lon.h"

struct PID_PARA pid_para;
float speed=0;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lon_controller_node");
    ros::NodeHandle n_handle;

    //->话题订阅 ：车辆姿态（RTK+IMU）、车辆状态（vehicle_chassis）
    ros::Subscriber sub_veh_statu   = n_handle.subscribe("VehicleStat",1,veh_status_callback);
    ros::Subscriber sub_veh_pose    = n_handle.subscribe("navi_msg",1,veh_pose_callback);
    ros::Subscriber sub_run_req     = n_handle.subscribe("Request",1,run_req_callback);

    //->话题发布 ：油门和刹车控制命令
    ros::Publisher  pub_AEB = n_handle.advertise<custom_msgs::AEBCmd>("AEBCmd",1);
    ros::Publisher  pub_trq_bre = n_handle.advertise<custom_msgs::TrqBreCmd>("TrqBreCmd",1);
    ros::Publisher  pub_Shift   = n_handle.advertise<custom_msgs::GearCmd>("GearCmd",1);
    custom_msgs::AEBCmd AEB_msg;
    custom_msgs::TrqBreCmd trq_bre_msg;
    custom_msgs::GearCmd shift_msg;

    n_handle.param("/error",pid_para.error,(float)0);
    n_handle.param("/error_last",pid_para.last_error,(float)0);
    n_handle.param("/KP",pid_para.KP,(float)0);
    n_handle.param("/KI",pid_para.KI,(float)0);
    n_handle.param("/KD",pid_para.KD,(float)0);
    n_handle.param("/speed",speed,request_value.run_speed);

    //-->>设置主线程循环周期 100ms
    ros::Rate loop_rate(200);

   //计算中间量
    struct AEB aeb ;
    struct Trq_Bre_Cmd temp_cmd = {0,0,0,0,0,0};
    struct AEB_Cmd AEB_temp_cmd = {0,0}; 
    

    while(ros::ok()){ 
      ros::spinOnce();

  
      switch (request_value.request_type) {
      case AEB_ENABLE:{  //AEB进行处理
          ROS_INFO_THROTTLE(1, "--------------AEB------------");
              aeb = aeb_solve();
              AEB_msg.AEB_enable      = aeb.Aeb_Cmd.AEB_enable;
              AEB_msg.AEB_bre_value   = aeb.Aeb_Cmd.AEB_bre_value;
              temp_cmd = aeb.Bre_Cmd;
              trq_bre_msg.bre_enable  = temp_cmd.bre_enable;
              trq_bre_msg.bre_value   = temp_cmd.bre_value;
              trq_bre_msg.trq_enable  = temp_cmd.trq_enable;
              trq_bre_msg.trq_value_3 = temp_cmd.trq_value;
              pub_trq_bre.publish(trq_bre_msg);
              pub_AEB.publish(AEB_msg);
              break;
            }
      case STOP_ENABLE:{           
         //停车请求处理
          ROS_INFO_THROTTLE(1, "---------------STOP-----------");
          shift_msg.APA_GearEnable   = 1;
          shift_msg.APA_GearRequest  = 0x00;    // P：:0x0；
          pub_Shift.publish(shift_msg);
          temp_cmd = stop_solve();
          AEB_msg.AEB_enable   = 0;
          AEB_msg.AEB_bre_value  = 0;
          trq_bre_msg.bre_value   = temp_cmd.bre_value;
          trq_bre_msg.bre_enable  = temp_cmd.bre_enable;
          trq_bre_msg.trq_enable  = temp_cmd.trq_enable;
          trq_bre_msg.trq_value_3 = temp_cmd.trq_value;
          pub_trq_bre.publish(trq_bre_msg);
          pub_AEB.publish(AEB_msg);
          break;
        }
      case FORWARD_ENABLE:{ //正常运行模式处理
          ROS_INFO_THROTTLE(1, "---------RUN----------------");
          shift_msg.APA_GearEnable   = 1;
          shift_msg.APA_GearRequest  = 0x03;     // D: 0x3
          pub_Shift.publish(shift_msg);
          //发布行驶过程中的油门刹车值
          temp_cmd = run_PID_solve();  
          AEB_msg.AEB_enable   = 0;
          AEB_msg.AEB_bre_value  = 0;
          trq_bre_msg.bre_enable  =temp_cmd.bre_enable;
          trq_bre_msg.bre_value   =temp_cmd.bre_value;
          trq_bre_msg.trq_enable  = temp_cmd.trq_enable;
          trq_bre_msg.trq_value_3 = temp_cmd.trq_value;
          pub_AEB.publish(AEB_msg);
          pub_trq_bre.publish(trq_bre_msg);
          break;
        }
      case BACK_ENABLE: { //正常运行模式处理
          ROS_INFO_THROTTLE(1, "----------------BACK---------");
          shift_msg.APA_GearEnable = 1;
          shift_msg.APA_GearRequest = 0x01;    //R :0x1  
          pub_Shift.publish(shift_msg);

          AEB_msg.AEB_enable = 0;
          AEB_msg.AEB_bre_value = 0;
          //发布倒车档过程中的油门刹车值
          temp_cmd=run_PID_solve() ;
          trq_bre_msg.bre_value   = temp_cmd.bre_value;
          trq_bre_msg.bre_enable  = temp_cmd.bre_enable;
          trq_bre_msg.trq_enable  = temp_cmd.trq_enable;
          trq_bre_msg.trq_value_3 = temp_cmd.trq_value;
          pub_AEB.publish(AEB_msg);
          pub_trq_bre.publish(trq_bre_msg);
          break;
      }
      }

        loop_rate.sleep();
    }
    return 0;
}
