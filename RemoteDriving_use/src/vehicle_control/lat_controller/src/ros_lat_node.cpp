#include "ros_interface_lat.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle n_handle;
    //->话题订阅 ：参考路径(Path planning)、车辆状态（vehicle_chassis）
    //ros::Subscriber sub_ref_path    = n_handle.subscribe("ref_planned_path",1,ref_path_callback);
    ros::Subscriber sub_ref_path    = n_handle.subscribe("raw_path",1,ref_path_callback);
    ros::Subscriber sub_veh_statu   = n_handle.subscribe("VehicleStat",1,veh_status_callback);
    ros::ServiceClient  SerClient_turnLight = n_handle.serviceClient<custom_msgs::TurnLightCmd>("TurnLightCmd");
    //ros::Subscriber sub_run_req     = n_handle.subscribe("Request",1,turn_light_callback);

    //->话题发布 ：方向控制命令
    ros::Publisher  pub_steer       = n_handle.advertise<custom_msgs::SteeringCmd>("SteeringCmd",1);
    //-->>设置主线程循环周期
    ros::Rate loop_rate(200);

    //->初始化车辆参数
    ros_veh_para_init();

    custom_msgs::SteeringCmd steer_msg;
    custom_msgs::TurnLightCmd TurnLight;

    while(ros::ok()){
      steer_msg.SteeringAngle = pure_pursuit();
      //cout<<"steer_msg.steer_angle = "<<steer_msg.SteeringAngle<<endl;
      ROS_INFO_THROTTLE(1, "steer_msg.steer_angle = %f",steer_msg.SteeringAngle);
      //TurnLight.request.TurnLightEnable = set_turn_light();
/*
      //处理汇流车道
      if(turn_light_enable == 0x1){
       TurnLight.request.TurnLightEnable = 0x2;
      }
*/
      //发布转向灯消息                                                                                                                                            
      //SerClient_turnLight.call(TurnLight);

      //转向消息
      pub_steer.publish(steer_msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
