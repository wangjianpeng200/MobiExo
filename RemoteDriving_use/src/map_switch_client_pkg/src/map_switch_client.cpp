#include <ros/ros.h>
#include <custom_msgs/Map_Switch.h>  // 引入服务消息类型

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_switch_client"); // 初始化ROS节点
    ros::NodeHandle nh;

    // 创建一个服务客户端，指定服务名称和服务类型
    ros::ServiceClient client = nh.serviceClient<custom_msgs::Map_Switch>("/map_switch_server");

    // 准备服务请求
    custom_msgs::Map_Switch srv;
    srv.request.id = 1; // 设置要切换的地图 ID

    // 调用服务并检查是否成功
    if (client.call(srv)) {
        if (srv.response.isSuccess) {
            ROS_INFO("Map switch successful!");
        } else {
            ROS_WARN("Map switch failed!");
        }
    } else {
        ROS_ERROR("Failed to call service /map_switch_server");
    }

    return 0;
}
