#include <iostream>
#include <vector>
#include <cstring>  // For memcpy
#include <arpa/inet.h> // For socket functions
#include <unistd.h> // For close()
#include <cstdlib> 
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

// 定义 pose 结构体
struct pose {
    double x;
    double y;
};

// 反序列化函数
std::vector<pose> deserialize(const std::vector<char>& buffer) {
    std::vector<pose> poses(buffer.size() / sizeof(pose));
    std::memcpy(poses.data(), buffer.data(), buffer.size());
    return poses;
}

// UDP 接收函数
std::vector<pose> receiveUDP(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    // server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    // server_addr.sin_addr.s_addr = inet_addr("192.168.1.4");
    server_addr.sin_addr.s_addr = inet_addr("192.168.3.60");

    server_addr.sin_port = htons(port);

    if (bind(sock, reinterpret_cast<sockaddr*>(&server_addr), sizeof(server_addr)) < 0) {
        perror("Bind failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    std::vector<char> buffer(1024); 
    sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);

    ssize_t received_bytes = recvfrom(sock, buffer.data(), buffer.size(), 0,
                                      reinterpret_cast<sockaddr*>(&client_addr), &addr_len);
    if (received_bytes < 0) {
        perror("Receive failed");
        close(sock);
        exit(EXIT_FAILURE);
    }

    buffer.resize(received_bytes);
    std::vector<pose> poses = deserialize(buffer);

    close(sock);
    return poses;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "udp_to_path");
    ros::NodeHandle n_handle;
            std::cout <<"hello()"<<std::endl;

    // 发布 nav_msgs/Path 消息
    ros::Publisher pub_path = n_handle.advertise<nav_msgs::Path>("/gps_path_topic", 1);
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map"; // 修改为你的坐标系

    int port = 1600; // 替换为你发送端使用的端口

    while (ros::ok()) {
        // 接收新的 UDP 数据
        std::vector<pose> received_poses = receiveUDP(port);
            std::cout <<"size : " << received_poses.size()<<std::endl;
        int count = 0;
        // 将新的 UDP 数据添加到 Path 消息中
        for (const auto& p : received_poses) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "map"; // 修改为你的坐标系
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose.position.x = p.x;
            pose_stamped.pose.position.y = p.y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0; // 假设没有旋转
            // std::cout <<" "<< p.x << " " << p.y << " count : " << count++ << std::endl;
            path_msg.poses.push_back(pose_stamped);
        }

        // 更新 Path 消息的时间戳
        path_msg.header.stamp = ros::Time::now();

        // 发布 Path 消息
        pub_path.publish(path_msg);

        ros::spinOnce();
    }

    return 0;
}
