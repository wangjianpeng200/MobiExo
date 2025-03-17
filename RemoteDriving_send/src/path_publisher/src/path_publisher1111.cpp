

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <algorithm> // For std::min and std::max
#include "string"
double max_ange = 45;
double L = 4.8;
double max_orien_ange = 580;
double tolerance_distance = 0.1;
bool path_completed = false;
template<typename T>
T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(hi, v));
}

class PurePursuit {
public:
    PurePursuit(ros::NodeHandle &nh)
        : last_lookahead_index_(0), robot_yaw(0.0) // 初始化 last_lookahead_index_ 为 0，robot_yaw 为 0
    {
        // 订阅车辆当前位置
        pose_sub_ = nh.subscribe("cur_pose", 1, &PurePursuit::poseCallback, this);
        // 发布路径和预瞄点位置
        path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        vehicle_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_topic", 10);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        gps_pose_sub_ = nh.subscribe("/gps_path_topic", 1, &PurePursuit::readgpsCallback, this);
        // 设置预瞄点的距离
        lookahead_distance_ = L / 2;
    }

    void readgpsCallback(const nav_msgs::Path::ConstPtr &msg) {
        path_points_.clear();
        for (size_t i = 0; i < msg->poses.size(); ++i) {
            const geometry_msgs::PoseStamped& pose = msg->poses[i];
            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            path_points_.emplace_back(x, y);
        }
        path_completed = false;
    }


    double computeAngle(double Ax, double Ay, double Bx, double By, double Cx, double Cy) {
        // 计算向量 AB 和 AC
        double ABx = Bx - Ax;
        double ABy = By - Ay;
        double ACx = Cx - Ax;
        double ACy = Cy - Ay;

        // 计算点积和向量长度
        double dot_product = ABx * ACx + ABy * ACy;
        double AB_length = sqrt(ABx * ABx + ABy * ABy);
        double AC_length = sqrt(ACx * ACx + ACy * ACy);

        // 计算余弦值并求出角度（弧度）
        double cos_theta = dot_product / (AB_length * AC_length);
        double angle = acos(cos_theta);  // 结果为 0 到 pi

        // 计算叉积以确定方向
        double cross = ABx * ACy - ABy * ACx;

        // 将弧度转换为角度
        angle = angle * 180.0 / M_PI;

        // 根据叉积确定角度的正负
        if (cross < 0) {
            angle = -angle;  // C 在 AB 的右侧，角度为负
        }

        // 限制角度范围在 -45 到 45 度
        return clamp(angle, -45.0, 45.0);
    }

    bool compute_reverse_forward() {
        // 获取当前车辆的朝向
        double dx = lookahead_point_.first - vehicle_pos_.x;
        double dy = lookahead_point_.second - vehicle_pos_.y;

        // 计算预瞄点和车辆位置之间的角度（相对于x轴）
        double angle_to_lookahead = atan2(dy, dx);

        // 计算该角度和车辆朝向的差
        double angle_difference = angle_to_lookahead - robot_yaw;

        // 将角度差限制在 -π 到 π 的范围内
        while (angle_difference > M_PI) angle_difference -= 2 * M_PI;
        while (angle_difference < -M_PI) angle_difference += 2 * M_PI;

        // 如果角度差的绝对值大于90度，则预瞄点在车辆后方
        return fabs(angle_difference) > M_PI / 2;
    }

    void compute_angle(const std::pair<double, double> &lookahead_point) {   
        bool back_val = compute_reverse_forward();
        cmd_msg.linear.x = back_val ? -1 : 1;

        double extension_x = vehicle_pos_.x + 3 * cos(robot_yaw) * cmd_msg.linear.x;
        double extension_y = vehicle_pos_.y + 3 * sin(robot_yaw) * cmd_msg.linear.x;
        double delta1 = computeAngle(vehicle_pos_.x, vehicle_pos_.y, extension_x, extension_y, lookahead_point.first, lookahead_point.second);
        delta1 = atan2(2 * L * sin(delta1 * M_PI / 180.0), lookahead_distance_) * 180 / M_PI;
        delta1 = clamp(delta1, -max_ange, max_ange);

        if (fabs(delta1) > 15 && fabs(delta1) <= 25) delta1 *= 22.2;
        else if (fabs(delta1) > 25 && fabs(delta1) <= 30) delta1 *= 19.33;
        else if (fabs(delta1) > 30 && fabs(delta1) <= 45) delta1 *= 12.00;
        else delta1 *= 8.88;
        if(fabs(delta1) > max_orien_ange) delta1 = (fabs(delta1) / delta1) * max_orien_ange;
        std::string back_forward;

        if (back_val) {
            cmd_msg.angular.z = -delta1;
            back_forward = "back";
            // ROS_WARN_THROTTLE(1, "back %f", cmd_msg.angular.z);
        } else {
            cmd_msg.angular.z = delta1;
            back_forward = "forward";
            // ROS_WARN_THROTTLE(1, "forward %f", cmd_msg.angular.z);
        }
        ROS_WARN_THROTTLE(1, "%s %f", back_forward.c_str(), cmd_msg.angular.z);

        cmd_vel_pub.publish(cmd_msg);
    }

    void compute_extension_point(double extension_distance) {
        // 计算沿当前 yaw 方向延长 extension_distance 的点
        double extension_x = vehicle_pos_.x + extension_distance * cos(robot_yaw);
        double extension_y = vehicle_pos_.y + extension_distance * sin(robot_yaw);

        // 可视化延长点
        visualization_msgs::Marker extension_marker;
        extension_marker.header.frame_id = "map";
        extension_marker.header.stamp = ros::Time::now();
        extension_marker.ns = "extension_point";
        extension_marker.id = 2;
        extension_marker.type = visualization_msgs::Marker::SPHERE;
        extension_marker.action = visualization_msgs::Marker::ADD;
        extension_marker.pose.position.x = extension_x;
        extension_marker.pose.position.y = extension_y;
        extension_marker.scale.x = 0.5;
        extension_marker.scale.y = 0.5;
        extension_marker.scale.z = 0.5;
        extension_marker.color.r = 0.0;
        extension_marker.color.g = 0.0;
        extension_marker.color.b = 1.0;
        extension_marker.color.a = 1.0;

        // 发布延长点标记
        marker_pub_.publish(extension_marker);
    }

    void poseCallback(const geometry_msgs::Pose2D::ConstPtr &msg) {
        vehicle_pos_ = *msg;
        updateLookaheadPoint();
        visualize();
    }

    void stopVehicle() {
        if (path_completed) {
            ROS_WARN("Path has already been completed, no further action.");
            return;
        }

        // 停止指令
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_pub.publish(stop_msg);

        ROS_WARN_THROTTLE(1, "Vehicle stopped at the final lookahead point.");

        // 清空路径点
        path_points_.clear();
        path_completed = true;
        ROS_INFO("Path points have been cleared.");
    }


    void updateLookaheadPoint() {
        for (size_t i = last_lookahead_index_; i < path_points_.size(); ++i) {
            double dx = path_points_[i].first - vehicle_pos_.x;
            double dy = path_points_[i].second - vehicle_pos_.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double distance1 = std::sqrt((path_points_[path_points_.size() - 1].first - vehicle_pos_.x)
             * (path_points_[path_points_.size() - 1].first - vehicle_pos_.x)
              + 
              (path_points_[path_points_.size() - 1].second - vehicle_pos_.y) 
              * (path_points_[path_points_.size() - 1].second - vehicle_pos_.y));
            ROS_WARN_THROTTLE(1, "%f", distance1);

            // 判断是否接近最后一个预瞄点
            if (i == path_points_.size() - 1 && distance1 < tolerance_distance) {
                // 停止车辆
                std::cout << " last into" << std::endl;
                stopVehicle();
                return;
            }
            if (distance > lookahead_distance_) {
                lookahead_point_ = path_points_[i];
                last_lookahead_index_ = i;
                compute_angle(lookahead_point_);
                break;
            }
        }
    }

    void publishPose(const geometry_msgs::Pose& pose) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position = pose.position;
        pose_stamped.pose.orientation = pose.orientation;
        vehicle_pose_pub.publish(pose_stamped);
    }

    void visualize()
    {
        // 可视化路径
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";

        for (const auto &point : path_points_)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = point.first;
            pose.pose.position.y = point.second;
            path_msg.poses.push_back(pose);
        }

        path_pub_.publish(path_msg);

        // 更新 base_link 的 yaw
        tf::StampedTransform transform;
        try
        {
            // 获取 base_link 到 map 坐标系的变换
            tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
            double roll, pitch, yaw;
            transform.getBasis().getRPY(roll, pitch, yaw);
            robot_yaw = yaw + M_PI / 2;  // 更新 robot_yaw
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Could not get transform: %s", ex.what());
        }

        // 设置示例 Pose
        geometry_msgs::Pose vehicle_pose;
        vehicle_pose.position.x = vehicle_pos_.x;  // X 位置
        vehicle_pose.position.y = vehicle_pos_.y;  // Y 位置
        vehicle_pose.position.z = 0.0;             // Z 位置

        // 设置朝向（四元数）
        tf2::Quaternion q;
        q.setRPY(0, 0, robot_yaw);  // 设置为朝向 90°（右侧）
        vehicle_pose.orientation.x = q.x();
        vehicle_pose.orientation.y = q.y();
        vehicle_pose.orientation.z = q.z();
        vehicle_pose.orientation.w = q.w();

        // 可视化预瞄点
        visualization_msgs::Marker lookahead_marker;
        lookahead_marker.header.frame_id = "map";
        lookahead_marker.header.stamp = ros::Time::now();
        lookahead_marker.ns = "lookahead_point";
        lookahead_marker.id = 1;
        lookahead_marker.type = visualization_msgs::Marker::SPHERE;
        lookahead_marker.action = visualization_msgs::Marker::ADD;
        lookahead_marker.pose.position.x = lookahead_point_.first;
        lookahead_marker.pose.position.y = lookahead_point_.second;
        lookahead_marker.scale.x = 0.5;
        lookahead_marker.scale.y = 0.5;
        lookahead_marker.scale.z = 0.5;
        lookahead_marker.color.r = 0.0;
        lookahead_marker.color.g = 1.0;
        lookahead_marker.color.b = 0.0;
        lookahead_marker.color.a = 1.0;

        publishPose(vehicle_pose);

        // 发布标记
        // marker_pub_.publish(vehicle_marker);
        marker_pub_.publish(lookahead_marker);

        compute_extension_point(3);
    }

private:
    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher vehicle_pose_pub;
    ros::Publisher cmd_vel_pub;

    ros::Subscriber gps_pose_sub_;

    geometry_msgs::Pose2D vehicle_pos_;
    std::vector<std::pair<double, double>> path_points_;
    std::pair<double, double> lookahead_point_;
    double lookahead_distance_;
    size_t last_lookahead_index_;
    tf::TransformListener tf_listener;
    double robot_yaw;
    geometry_msgs::Twist cmd_msg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_gps");
    ros::NodeHandle nh;
    PurePursuit pure_pursuit(nh);
    ros::spin();
    return 0;
}

