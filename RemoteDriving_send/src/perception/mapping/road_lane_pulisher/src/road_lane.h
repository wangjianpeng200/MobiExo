#ifndef ROAD_LANE_H
#define ROAD_LANE_H

#include <ros/ros.h>
#include <custom_msgs/LaneLine.h>
#include <custom_msgs/LaneLineArray.h>
#include <custom_msgs/CurPose.h>
#include <custom_msgs/Map_Switch.h>
#include <compute/map.h>
#include <compute/frenet.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>

typedef struct inter_section
{
    bool IsInterSection; 
    double longitude;               
    double latitude;                  
    double longitude_stop_line;       
    double latitude_stop_line;        
}InterSection;

typedef struct tunnel
{
    bool IsTunnel;  
}Tunnel;

typedef struct current_lane  //获取车道线信息
{
    int nextlane;
    std::vector<LanePoint> left_road_points;
    std::vector<LanePoint> right_road_points;
    std::vector<LanePoint> refer_road_points;
    double width;
    LanePoint start_points;
    LanePoint end_points;
    InterSection pass_inter_section;
    Tunnel pass_tunnel;



}CurLane;

class RoadLane
{
public:
    RoadLane();
    int exec();
private:
    ros::NodeHandle nh;
    ros::Subscriber cur_pose_sub, running_status_sub;
    ros::Publisher road_lane_pub, cur_pose_sd_pub, cur_scene_pub, velocity_pub;
    ros::ServiceClient turn_light_client;
    ros::ServiceClient avoid_ambulance_client;
    Lane_Message cur_lane;
    //int turn_light_num, map_type = 0;
    int last_map_type = 0;
    custom_msgs::LaneLine refer_road_lane;
    custom_msgs::LaneLine left_road_lane, right_road_lane;
    //每条道路起点s
    std::vector<double> s_start;
    //每条道路终点s
    std::vector<double> s_end;

    ros::ServiceServer map_srv;
    bool map_switch(custom_msgs::Map_Switch::Request &req,
                    custom_msgs::Map_Switch::Response &res);



    vector<LanePoint> LanePoints;

    bool IsInit, IsGetAvoidLane;
    void CurPoseRecvd(const geometry_msgs::Pose2D &msg);
    void GetRoadLane(const int &lane_id);
    void IsLaneInit(const geometry_msgs::Pose2D &msg);
    void getAllLaneBEP(int laneCnt);


    void GetAoidRoadLane();
    void init_s_size(int);
    void cal_s_value(int);

};



#endif 