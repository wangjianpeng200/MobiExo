#include "road_lane.h"

RoadLane::RoadLane()
{
    map_srv = nh.advertiseService("/map_switch_server", &RoadLane::map_switch,this); //发布地图id 切换服务

    road_lane_pub = nh.advertise<custom_msgs::LaneLineArray>("/road_lane", 1, true);
    cur_pose_sd_pub = nh.advertise<custom_msgs::CurPose>("/cur_pose_all", 1, true);
    //velocity_pub = nh.advertise<geometry_msgs::Vector3>("/map_velocity", 1,true);
    //turn_light_num = 1;
    IsInit = true;


    std::string cur_pose_topic;
    nh.param<std::string>("/node_road_lane/cur_pose_topic", cur_pose_topic,"/cur_pose");
    cur_pose_sub = nh.subscribe(cur_pose_topic,1,&RoadLane::CurPoseRecvd,this);     ///cur_pose才会触发地图发布

}

bool RoadLane::map_switch(custom_msgs::Map_Switch::Request &req,
         custom_msgs::Map_Switch::Response &res){
    GetRoadLane(req.id);
    res.isSuccess = true;
    return true;
}

void RoadLane::GetRoadLane(const int &lane_id)
{
    int last_id = lane_id;
    refer_road_lane.x.clear();
    refer_road_lane.y.clear();
    refer_road_lane.s.clear();
    left_road_lane.x.clear();
    left_road_lane.y.clear();
    right_road_lane.x.clear();
    right_road_lane.y.clear();
    custom_msgs::LaneLineArray road_lane;
    double sum_refer_s = 0;
    cur_lane = select_Lane_by_id(lane_id);
    int cur_refer_line_size = cur_lane.ReferencePoints_current_line.size();
    // refer_road_lane.x.push_back(cur_lane.ReferencePoints_current_line[0].x);
    // refer_road_lane.y.push_back(cur_lane.ReferencePoints_current_line[0].y);

    for(int i = 0; i < cur_refer_line_size; i++)    //参考点 xy
    {
        refer_road_lane.x.push_back(cur_lane.ReferencePoints_current_line[i].x);
        refer_road_lane.y.push_back(cur_lane.ReferencePoints_current_line[i].y);
    }

    for(int i = 0; i < cur_refer_line_size; i++)     //参考点 sd
    {
        sum_refer_s = getFrenet2(cur_lane.ReferencePoints_current_line[i].x,cur_lane.ReferencePoints_current_line[i].y,
                                    refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id])[0];
        refer_road_lane.s.push_back(sum_refer_s);
    }

    refer_road_lane.id = 0;
    refer_road_lane.type = 0;
    refer_road_lane.current_lane_num = cur_lane.current_lane_id;
    int cur_left_line_size = cur_lane.LanePoints_left_current_line.size();
    for(int i = 0; i < cur_left_line_size; i++)     //左点
    {
        left_road_lane.x.push_back(cur_lane.LanePoints_left_current_line[i].x);
        left_road_lane.y.push_back(cur_lane.LanePoints_left_current_line[i].y);
    }
    left_road_lane.id = 1;
    left_road_lane.type = 1;
    int cur_right_line_size = cur_lane.LanePoints_right_current_line.size();
    for(int i = 0; i < cur_right_line_size; i++)    //右点
    {
        right_road_lane.x.push_back(cur_lane.LanePoints_right_current_line[i].x);
        right_road_lane.y.push_back(cur_lane.LanePoints_right_current_line[i].y);
    }
    right_road_lane.id = 2;
    right_road_lane.type = 1;
    road_lane.lines.push_back(refer_road_lane);
    road_lane.lines.push_back(left_road_lane);
    road_lane.lines.push_back(right_road_lane);
    road_lane_pub.publish(road_lane);           //道路发布

    if(abs(cur_lane.current_lane_id) <= ID_SUM && abs(cur_lane.current_lane_id)>0){
         cout<<"____________road_lane publish OK!!!"<<endl;
         std::cout << "________Current_lane_id: " << cur_lane.current_lane_id <<std::endl;
         std::cout << "___________Next_lane_id: " << cur_lane.next_lane_id <<std::endl;
    }else{

         ROS_WARN("road_lane publish falied! Please check map_origin set is right? or Gnss_localization is right?");
    }
}

//初始化两个s数组的大小
void RoadLane::init_s_size(int laneCnt){
    s_start.resize(laneCnt+10);
    s_end.resize(laneCnt+10);
}

void RoadLane::cal_s_value(int laneCnt){
    s_start[0] = 0;
    s_end[0] = 0;
    s_start[1] = 0;
    for(int i = 1; i <= laneCnt; i++){
        LanePoints.clear();
        GetLaneBEP(i, LanePoints);
        if(LanePoints.size() == 0){
            s_end[i] = s_start[i];
            s_start[i+1] = s_end[i];
            continue;
        }

        std::vector<double> map_x;
        std::vector<double> map_y;
        for(int i = 0; i < LanePoints.size();i++){
    
            map_x.push_back(LanePoints[i].x);
            map_y.push_back(LanePoints[i].y);
        }
        s_end[i] = getFrenet2(map_x[map_x.size()-1],map_y[map_y.size()-1],map_x,map_y,s_start[i])[0];
        s_start[i+1] = s_end[i];

    }
}

void RoadLane::IsLaneInit(const geometry_msgs::Pose2D &msg)
{
    if(IsInit)
    {
        //初始化每一个s的数组
        int laneCnt = GetLaneCnt();
        init_s_size(laneCnt);
        cal_s_value(laneCnt);
        
        int cur_id = Get_Lane_id_by_Coor(msg.x, msg.y); //根据xy判断哪段map
        IsInit = false;
        GetRoadLane(cur_id);    //根据id发布
    }
}


void RoadLane::CurPoseRecvd(const geometry_msgs::Pose2D &msg)
{
    
    IsLaneInit(msg);    //通过当前x，y判断该加载哪段地图， 并完成该段地图加载
 
    std::vector<double> start_frenet = getFrenet2(cur_lane.current_lane_start_point.x,cur_lane.current_lane_start_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
    
    std::vector<double> end_frenet = getFrenet2(cur_lane.current_lane_end_point.x, cur_lane.current_lane_end_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]); 
  
    std::vector<double> cur_frenet = getFrenet2(msg.x,msg.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
   
    if((cur_frenet[0]+6 >= end_frenet[0]) && IsInit==false)
    {
        GetRoadLane(cur_lane.next_lane_id);
        start_frenet = getFrenet2(cur_lane.current_lane_start_point.x,cur_lane.current_lane_start_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);
        end_frenet = getFrenet2(cur_lane.current_lane_end_point.x, cur_lane.current_lane_end_point.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]); 
        cur_frenet = getFrenet2(msg.x,msg.y,refer_road_lane.x,refer_road_lane.y,s_start[cur_lane.current_lane_id]);

    }  

    custom_msgs::CurPose cur_pose_all;
    cur_pose_all.x = msg.x;
    cur_pose_all.y = msg.y;
    cur_pose_all.theta = msg.theta;
    cur_pose_all.s = cur_frenet[0];
    cur_pose_all.d = cur_frenet[1];

    cur_pose_sd_pub.publish(cur_pose_all);  //发布sd

}

int RoadLane::exec()
{
    ros::spin();
    return 0;
}


