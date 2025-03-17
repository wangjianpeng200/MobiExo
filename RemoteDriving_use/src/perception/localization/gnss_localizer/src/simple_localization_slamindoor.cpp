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
#include <geometry_msgs/PoseStamped.h>
#include <custom_msgs/SlamPose.h>
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

    tf::StampedTransform transform_slam;

    double map_longitude;
    double map_latitude;
    float map_yaw;

    geometry_msgs::Pose2D cur_pose;

    float theta;
    VehPose pos_map;
    CoordPoint map_ori;

	ros::Subscriber trackedpose_sub;
	ros::Publisher slam_pose_pub;
	custom_msgs::SlamPose slam_pose;

private:
    void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr& msg);
    void onSlamMsgRecvd(const ::geometry_msgs::PoseStamped& msg); 

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


//pure slam
SimpleLocalization::SimpleLocalization()
{

    chatter_pub = nh.advertise<geometry_msgs::Pose2D>("cur_pose", 1);
    speed_pub = nh.advertise<std_msgs::Float32>("cur_speed", 1); 
   // sub = nh.subscribe("navi_msg",1,&SimpleLocalization::onNaviMsgRecvd,this);    //无惯导 无法进入
    trackedpose_sub = nh.subscribe("tracked_pose",1,&SimpleLocalization::onSlamMsgRecvd,this);  //slam位姿获取
	slam_pose_pub = nh.advertise<custom_msgs::SlamPose>("slam_pose", 1); 

    nh.param<bool>("/gnss_localizer_node/is_display_trajectory",is_display_trajectory,false);
    nh.param<bool>("/gnss_localizer_node/is_save_trajectory",is_save_trajectory,false);//保存实时运动轨迹

    if(is_save_trajectory){
        const char* env_p = std::getenv("SEED_HOME");
        if(env_p == NULL){
            ROS_FATAL("exception happened while reading env variable \"SEED_HOME\" ");
            exit(1);
        }
        std::string home_path = env_p;
        fout.open(home_path+"/data/slam/real_time_trajectory.txt",std::ios::out);
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
    ros::spin();
}

void SimpleLocalization::onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg)
{
    std_msgs::Float32 speed;
    speed.data = msg->speed2d;
    speed_pub.publish(speed);  


    try{
        //得到坐标map和坐标imu之间的关系
      listener.waitForTransform("map", "imu", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("map", "imu",
                               ros::Time(0), transform_slam);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }


    cur_pose.x=transform_slam.getOrigin().x();
    cur_pose.y=transform_slam.getOrigin().y(); 

    //1
    // cur_pose.theta = tf::getYaw(transform_slam.getRotation()) * 180/M_PI + 90;
   //2
    cur_pose.theta = tf::getYaw(transform_slam.getRotation()) * 180/M_PI;
    cur_pose.theta = -cur_pose.theta;
    cur_pose.theta = (cur_pose.theta < 0) ? (360 + cur_pose.theta) : (cur_pose.theta + 0);      // /navi_msg:heading 
    cur_pose.theta = (cur_pose.theta >= 0 && cur_pose.theta < 90) ?  (90 - cur_pose.theta) : (450 - cur_pose.theta); 
 
    chatter_pub.publish(cur_pose);

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

        path_marker.color = red;        //实时轨迹显示颜色
       
        // if (42 == msg->pose_type )
        //     path_marker.color = lawngreen;
        // else if (52 == msg->pose_type)
        //     path_marker.color = cyan;
        // else
        //     path_marker.color = red;

        path_pub.publish(path_marker);

        if(is_save_trajectory)
            fout << p.x << " " << p.y << std::endl;
    
    }

    ros::spinOnce();
}

void SimpleLocalization::onSlamMsgRecvd(const ::geometry_msgs::PoseStamped& msg){

//发布速度
    std_msgs::Float32 speed;
  //  speed.data = msg->speed2d;
    speed.data = 1.44;     //与控制写死的速度保持一致
    speed_pub.publish(speed);  

//slam 姿态
	slam_pose.x = msg.pose.position.x;
	slam_pose.y = msg.pose.position.y;
	slam_pose.theta = tf::getYaw(msg.pose.orientation)*180/M_PI;    //角度

    //1
    slam_pose.theta = -slam_pose.theta;
    slam_pose.theta = (slam_pose.theta < 0) ? (360 + slam_pose.theta) : (slam_pose.theta + 0);      // /navi_msg:heading 
    slam_pose.theta = (slam_pose.theta >= 0 && slam_pose.theta < 90) ?  (90 - slam_pose.theta) : (450 - slam_pose.theta);    // /cur_pose: theta

    //2    
//	slam_pose.theta = tf::getYaw(msg.pose.orientation)*180/M_PI + 90;  

  	slam_pose_pub.publish(slam_pose);

//填充/cur_pose，发布
    cur_pose.x = slam_pose.x ;
    cur_pose.y = slam_pose.y ;
    cur_pose.theta = slam_pose.theta;
    chatter_pub.publish(cur_pose);


   if(is_display_trajectory){      //显示（保存）center_back坐标系原点在map坐标系中的轨迹
        try{
            listener.waitForTransform("map", "center_back", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("map", "center_back",
                                    ros::Time(0), transform);
        }catch (tf::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        geometry_msgs::Point p;
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y(); 
        p.z = 0;
        path_marker.points.push_back(p);

		if(path_marker.points.size() > 300)
            path_marker.points.erase(path_marker.points.begin());

        path_marker.color = lawngreen;        //实时轨迹显示颜色

        path_pub.publish(path_marker);

        if(is_save_trajectory)
            fout << p.x << " " << p.y << std::endl;
    
    }

    ros::spinOnce();


}


