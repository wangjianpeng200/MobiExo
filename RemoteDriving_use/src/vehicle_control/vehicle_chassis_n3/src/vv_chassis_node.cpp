#include "v_ros_msg_interface.h"

int main(int argc, char *argv[]){
    kp=ki=kd=0;
    ros::init(argc, argv, "vehicle_chassis_node");
    ros::NodeHandle n_handle;

    //-->>订阅话题 ： 转向控制命令、速度控制命令
    ros::Subscriber sub_Steering     = n_handle.subscribe("SteeringCmd",1,get_steering_cmd_callback);
    ros::Subscriber sub_TrqBre       = n_handle.subscribe("TrqBreCmd",1,get_trq_bre_cmd_callback);
    ros::Subscriber sub_AEBCmd       = n_handle.subscribe("AEBCmd",1,get_AEB_cmd_callback);
    ros::Subscriber sub_GearCmd      = n_handle.subscribe("GearCmd",1,get_Gear_cmd_callback);
    
    //-->>发布话题 ： 车辆状态
    ros::Publisher  pub_veh_statu = n_handle.advertise<custom_msgs::VehicleStat>("VehicleStat", 1);
    ros::Publisher  pub_TurnLight_statu = n_handle.advertise<custom_msgs::TurnLightStat>("TurnLightStatus", 1);

    n_handle.param("/Kp",kp,(float)0);
    n_handle.param("/Ki",ki,(float)0);
    n_handle.param("/Kd",kd,(float)0);

    custom_msgs::VehicleStat VehicleStatus_msg;
    custom_msgs::TurnLightStat TurnlightStatus_msg;

    struct VehicleStat VehicleStat;
    struct TurnLight_status TurnLight_status;

  ///--------------------------------------------------------------------------------------------------------------------------
  /// -------------------------------------------------------------------------------------------------------------------------
    //----->>初始化CAN卡
    //-->>条件编译
    // #ifdef NO_LOCAL_DEBUG1
    canid_t filter_canid[] = {ID3f1,ID3f2,ID3f3};
    can_fd = socketcan_init("can0",LOOPBACK_RECV_OWN::LOOPBACK_RECV_OWN_OFF_OFF);
    socketcan_filter_set(can_fd,filter_canid,sizeof(filter_canid)/sizeof(canid_t));
    // #endif

    //---->>开启获取车辆状态线程 开启命令发送定时器
    pthread_t client_thread_id = start_status_monitor_thread();
    start_send_timer(20);

  ///--------------------------------------------------------------------------------------------------------------------------
  /// -------------------------------------------------------------------------------------------------------------------------
    while(ros::ok()){
      //-->>获取车辆参数
      get_veh_status(VehicleStat);
      //-->>发布车辆状态
      VehicleStatus_msg.veh_speed                  = VehicleStat.VehicleSpeed;
      VehicleStatus_msg.SAS_SteeringAngle          = VehicleStat.SAS_SteeringAngle;
      VehicleStatus_msg.EMS_BrakePedalStatus       = VehicleStat.VcuBrkPedlSts;
      VehicleStatus_msg.TCU_GearShiftPositon       = VehicleStat.TCU_GearShiftPositon;
      //-->>获取车辆转向灯信息
      get_TurnLight_status(TurnLight_status);
      TurnlightStatus_msg.Veh_LowHighLight            = TurnLight_status.Veh_LowHighLight;
      TurnlightStatus_msg.Veh_TurnLightLe             = TurnLight_status.Veh_TurnLightLe ;
      TurnlightStatus_msg.Veh_BrakeLight              = TurnLight_status.Veh_BrakeLight ;
      TurnlightStatus_msg.Veh_VU_Horn                 = TurnLight_status.Veh_VU_Horn ;
      pub_TurnLight_statu.publish(TurnlightStatus_msg);
      pub_veh_statu.publish(VehicleStatus_msg);

      ros::spinOnce();

      usleep(1000*200);
    }
    //->取消线程    ->关闭CAN卡
    pthread_cancel(client_thread_id);
    close(can_fd);
    return 0;
}


