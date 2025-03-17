#include <iostream>
#include "ros/ros.h"
#include <vector>
#include <cstring>  
#include <arpa/inet.h> 
#include <unistd.h>  
#include "custom_msgs/CurPose.h"
#include <geometry_msgs/Pose2D.h>  // 使用 geometry_msgs/Pose2D

struct pose 
{
    double x;
    double y;
};

std::vector<pose> v_pos;
int cout = 0;

// 序列化函数
std::vector<char> serialize(const std::vector<pose>& poses) {
    std::vector<char> buffer(poses.size() * sizeof(pose));
    std::memcpy(buffer.data(), poses.data(), buffer.size());
    return buffer;
}

// UDP 发送函数
void sendUDP(const std::string& ip, int port) {
    // 创建套接字
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        return;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr);

    // 序列化数据
    std::vector<char> data = serialize(v_pos);

    // 发送数据
    ssize_t sent_bytes = sendto(sock, data.data(), data.size(), 0,
                                reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr));
    if (sent_bytes < 0) {
        perror("Send failed");
    } else {
        std::cout << "Sent " << sent_bytes << " bytes" << std::endl;
    }

    close(sock);
}

void onCurrentPoseMsgRecvd(const geometry_msgs::Pose2D::ConstPtr& msg)  
{   
    pose p;
    cout++;
    p.x = msg->x;
    p.y = msg->y;
    v_pos.push_back(p);
    // if(cout%2==0)
    // {
    //     sendUDP("192.168.3.83", 1500); 
    //     v_pos.clear();
    // }
    // sendUDP("192.168.1.7", 1600); 
    sendUDP("192.168.218.1", 1600); 

    v_pos.clear();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "udp");
    ros::NodeHandle n_handle;
    ros::Subscriber sub_current_pose = n_handle.subscribe("/cur_pose",1,onCurrentPoseMsgRecvd); 
    ros::spin();  
    return 0;
}