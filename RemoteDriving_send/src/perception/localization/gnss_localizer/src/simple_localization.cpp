#include "ros/ros.h"

#include <custom_msgs/NaviData.h>
#include <compute/trans_coord.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
    tf::TransformBroadcaster broadcaster;
    ros::Publisher path_pub;
    visualization_msgs::Marker path_marker;
	
	tf::TransformListener listener;
	tf::StampedTransform transform;

    double map_longitude;
    double map_latitude;
    float map_yaw;

    geometry_msgs::Pose2D cur_pose;
    float theta;
    VehPose pos_map;
    CoordPoint map_ori;
private:
    void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr& msg);
    void getPoseonMap(const custom_msgs::NaviData::ConstPtr& msg);

private:
    Eigen::AngleAxisd rollAngle;
    Eigen::AngleAxisd pitchAngle;
    Eigen::AngleAxisd yawAngle;
    Eigen::Quaterniond quaternion;
    std_msgs::ColorRGBA white;
    std_msgs::ColorRGBA lawngreen;
    std_msgs::ColorRGBA cyan;
	std_msgs::ColorRGBA red;

    bool is_display_trajectory;     //是否显示轨迹
    bool is_save_trajectory;        //是否保存轨迹
    std::fstream fout;
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
    sub = nh.subscribe("navi_msg",1,&SimpleLocalization::onNaviMsgRecvd,this);

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
        white.b = 0;
		
		red.a = 1;
        red.r = 1;
        red.g = 0;
        red.b = 0;
        
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
    try{
      listener.waitForTransform("center_back", "imu", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("center_back", "imu",
                               ros::Time(0), transform);
    }catch (tf::TransformException &ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    //x=transform.getOrigin().x();
    //y=transform.getOrigin().y();
    //z=transform.getOrigin().z();
    //cout<<"x: "<<x<<endl;
    //cout<<"y: "<<y<<endl;
    //cout<<"z: "<<z<<endl;

    ros::spin();
}

void SimpleLocalization::onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg)
{
    std_msgs::Float32 speed;
    speed.data = msg->speed2d;
    speed_pub.publish(speed);

    getPoseonMap(msg);

    yawAngle = Eigen::AngleAxisd((theta*M_PI)/180,Eigen::Vector3d::UnitZ());
    quaternion=yawAngle*pitchAngle*rollAngle;
    broadcaster.sendTransform(tf::StampedTransform(
                                  tf::Transform(tf::Quaternion(quaternion.x(),
                                                               quaternion.y(),
                                                               quaternion.z(),
                                                               quaternion.w()),
                                                tf::Vector3(cur_pose.x, cur_pose.y, 0)).inverse(),
                                  ros::Time::now(),"imu", "map"));
    chatter_pub.publish(cur_pose);
    // std::cout << "-------------------11111111111111111111------------------" << std::endl;

    if(is_display_trajectory){//显示（保存）center_back坐标系原点在map坐标系中的轨迹
        try{
            listener.waitForTransform("map", "center_back", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("map", "center_back",
                                    ros::Time(0), transform);
        }catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        geometry_msgs::Point p;
        // p.x = cur_pose.x ;
        // p.y = cur_pose.y ;
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        p.z = 0;
        path_marker.points.push_back(p);

		if(path_marker.points.size() > 500)
            path_marker.points.erase(path_marker.points.begin());
        if (42 == msg->pose_type )
            path_marker.color = lawngreen;
        else if (52 == msg->pose_type)
            path_marker.color = cyan;
        else
            path_marker.color = red;
        path_pub.publish(path_marker);

        if(is_save_trajectory)
            fout<<p.x<<" "<<p.y<<std::endl;
    }
    ros::spinOnce();
}

void SimpleLocalization::getPoseonMap(const custom_msgs::NaviData::ConstPtr &msg)
{
    VehPose pose_gps(msg->longitude,msg->latitude,msg->heading);
    trans_pose_gps2coord(map_ori,pose_gps,pos_map);

    cur_pose.x = pos_map.x_lon;
    cur_pose.y = pos_map.y_lat;
    cur_pose.theta = pos_map.yaw_heading;
    theta = map_yaw - msg->heading;
}
