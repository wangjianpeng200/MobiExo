#include "ros/ros.h"
#include <custom_msgs/UWBPose.h>
#include <custom_msgs/NaviData.h>
#include <compute/trans_coord.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <fstream>
#include <cstdlib>

class SimpleLocalization{

public:
    SimpleLocalization();
public:
    void exec();

private:
    ros::NodeHandle nh;
    ros::Publisher chatter_pub;
    ros::Publisher speed_pub;
    ros::Subscriber sub;
    ros::Subscriber uwb_positioning_sub;
    tf::TransformBroadcaster broadcaster;
    ros::Publisher path_pub;
    visualization_msgs::Marker path_marker;

    double map_longitude;
    double map_latitude;
    float map_yaw;

    double uwb_x = 0.0;
    double uwb_y = 0.0;

    geometry_msgs::Pose2D cur_pose;
    float theta;
    VehPose pos_map;
    CoordPoint map_ori;
    //UWB
    int no_navi_num = 0; 
    int navi_num = 100;
    int flag = 0;
	geometry_msgs::Point p;
private:
    void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr& msg);
    void getPoseonMap(const custom_msgs::NaviData::ConstPtr& msg);
    void onUWBMsgRecvd(const custom_msgs::UWBPose::ConstPtr& msg);

private:
    Eigen::AngleAxisd rollAngle;
    Eigen::AngleAxisd pitchAngle;
    Eigen::AngleAxisd yawAngle;
    Eigen::Quaterniond quaternion;
    std_msgs::ColorRGBA white;
    std_msgs::ColorRGBA lawngreen;
    std_msgs::ColorRGBA cyan;

    bool is_display_trajectory;     //是否显示轨迹
    bool is_save_trajectory;        //是否保存轨迹
    std::fstream fout;
    double time_n = 0.02;   //UWB
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_localization");
  SimpleLocalization loc;
  loc.exec();
  return 0;
}

SimpleLocalization::SimpleLocalization()
{
    rollAngle = Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX());
    pitchAngle = Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY());
    if(ros::param::has("~map_longitude"))
        ros::param::get("~map_longitude",map_longitude);
    else{
        ROS_WARN("ros param map_longitude not found, the localoization node will not work");
    }
    if(ros::param::has("~map_latitude"))
        ros::param::get("~map_latitude",map_latitude);
    else{
        ROS_WARN("ros param map_latitude not found, the localoization node will not work");
    }
    if(ros::param::has("~map_yaw"))
        ros::param::get("~map_yaw",map_yaw);
    else{
        ROS_WARN("ros param map_latitude not found, the localoization node will not work");
    }

    map_ori.x_lon = map_longitude;
    map_ori.y_lat = map_latitude;
    chatter_pub = nh.advertise<geometry_msgs::Pose2D>("cur_pose", 1);
    speed_pub = nh.advertise<std_msgs::Float32>("cur_speed", 1);
    sub = nh.subscribe("navi_msg",1,&SimpleLocalization::onNaviMsgRecvd,this);     //订阅惯导的话题

    uwb_positioning_sub = nh.subscribe("uwb_positioning",1,&SimpleLocalization::onUWBMsgRecvd,this);    //订阅UWB的话题

    nh.param<bool>("/gnss_localizer_node/is_display_trajectory",is_display_trajectory,false);
    nh.param<bool>("/gnss_localizer_node/is_save_trajectory",is_save_trajectory,false);

    if(is_save_trajectory){
        const char* env_p = std::getenv("SEED_HOME");
        if(env_p == NULL){
            ROS_FATAL("exception happened while reading env variable \"SEED_HOME\" ");
            exit(1);
        }
        std::string home_path = env_p;
        fout.open(home_path+"/data/navigation/trajectory.txt",std::ios::out);
    }

    if(is_display_trajectory){
        white.a = 1;
        white.r = 1;
        white.g = 1;
        white.b = 1;
        
        lawngreen.a = 1;
        lawngreen.r = 124.0/255;
        lawngreen.g = 252.0/255;
        lawngreen.b = 1;
        
        cyan.a = 1;
        cyan.r = 0;
        cyan.g = 1;
        cyan.b = 1;
        path_pub = nh.advertise<visualization_msgs::Marker>("trajectory", 1);
        path_marker.ns = "trajectory";
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.header.frame_id = "map";
        path_marker.scale.x = 0.1;
        path_marker.color = white;
        path_marker.lifetime = ros::Duration(0);
    }
}

void SimpleLocalization::exec()
{
    ros::spin();
}

//UWB回调函数
void SimpleLocalization::onUWBMsgRecvd(const custom_msgs::UWBPose::ConstPtr &msg)
{
    uwb_x = msg->x;
    uwb_y = msg->y;
    flag = msg->flag;
}

void SimpleLocalization::onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg)
{
    std_msgs::Float32 speed;
    speed.data = msg->speed2d;
    speed_pub.publish(speed);

    getPoseonMap(msg);  //经纬度转xy

    yawAngle = Eigen::AngleAxisd((theta*M_PI)/180,Eigen::Vector3d::UnitZ());
    quaternion=yawAngle*pitchAngle*rollAngle;

	if(42 == msg->pose_type || 0 == flag){      //42 52什么状态  有定位？？？？？？？？？？
		if(no_navi_num > 0){
			no_navi_num--;			
		}
    }else{                            //无定位？？？？？？？？？？？
        if(no_navi_num < navi_num){   
            no_navi_num++;
        }
    }
    //惯导和UWB位置权重求和
    p.x = (cur_pose.x*(navi_num-no_navi_num) + uwb_x * (no_navi_num))/(double)navi_num;
    p.y = (cur_pose.y*(navi_num-no_navi_num) + uwb_y * (no_navi_num))/(double)navi_num;
    //这一段？？？？？？？？？？？？？？？？？？？
    broadcaster.sendTransform(tf::StampedTransform(
                                  tf::Transform(tf::Quaternion(quaternion.x(),
                                                               quaternion.y(),
                                                               quaternion.z(),
                                                               quaternion.w()),
                                                tf::Vector3(p.x, p.y, 0)).inverse(),
                                  ros::Time::now(),"imu", "map"));
	cur_pose.x = p.x;
	cur_pose.y = p.y;
    chatter_pub.publish(cur_pose);
    if(is_display_trajectory){
		//p.x = cur_pose.x;
        //p.y = cur_pose.y;

		flag = 0;
        p.z = 0;
        path_marker.points.push_back(p);
        if (42 == msg->pose_type)
            path_marker.color = lawngreen;
        else
            path_marker.color = white;
        path_pub.publish(path_marker);
        //std::cout << 123123 << std::endl;
        if(is_save_trajectory)
            fout<< time_n<<" "<<p.x<<" "<<p.y<<std::endl;
    }
    ros::spinOnce();
}

void SimpleLocalization::getPoseonMap(const custom_msgs::NaviData::ConstPtr &msg)
{
    VehPose pose_gps(msg->longitude,msg->latitude,msg->heading);
    trans_pose_gps2coord(map_ori,pose_gps,pos_map);   //坐标原点，经纬度，存放转换后的xy

    cur_pose.x = pos_map.x_lon;
    cur_pose.y = pos_map.y_lat;
    cur_pose.theta = pos_map.yaw_heading;
    theta = map_yaw - msg->heading;    //map_yaw？？？？（地图偏转？）
}
