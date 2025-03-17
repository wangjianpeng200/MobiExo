#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>

#include <ros/ros.h>
#include <ros/time.h>
#include "custom_msgs/LidarRawObjectArray.h"
#include "custom_msgs/LidarRawObject.h"
#include "custom_msgs/RadarRawObjectArray.h"
#include "custom_msgs/RadarRawObject.h"

#include <visualization_msgs/Marker.h>

#include <cmath>
#define PI 3.1415926

using namespace std;

ofstream fout_lidar("/home/nvidia/Desktop/seed_veh4/txt/lidar_data.txt");
ofstream fout_radar("/home/nvidia/Desktop/seed_veh4/txt/radar_data.txt");

struct timeval tv;

ros::Publisher marker_pub;

int id1=0,id2=0;


void chatterCallback_lidar(const custom_msgs::LidarRawObjectArrayConstPtr& lidar_data)
{
    int len = lidar_data->objs.size();
    gettimeofday(&tv, NULL);

    for(int i=0 ; i<len ;i++){
        fout_lidar << id1++ <<" "<< tv.tv_sec << "." << tv.tv_usec <<" "<<lidar_data->objs[i].x_pos<<" "<< lidar_data->objs[i].y_pos <<" "<< lidar_data->objs[i].lwh.x<<" "<< lidar_data->objs[i].lwh.y <<" "<< lidar_data->objs[i].lwh.z<<endl;
    }
    
}


void chatterCallback_radar(const custom_msgs::RadarRawObjectArrayConstPtr& radar_data)
{
    int len = radar_data->objs.size();
    gettimeofday(&tv, NULL);

        ros::Rate r(5);
        float f = 0.0;
	//创建一个 visualization_msgs/Marker消息，并且初始化所有共享的数据。消息成员默认为0，仅仅设置位姿成员w。
	    visualization_msgs::Marker points;
	    points.header.frame_id = "radar";
	    points.header.stamp = ros::Time::now();
	    points.ns = "bag_to_txt";
	    points.action = visualization_msgs::Marker::ADD;
	    points.pose.orientation.w = 1.0;

	//创建一个 visualization_msgs/Marker消息，并且初始化所有共享的数据。消息成员默认为0，仅仅设置位姿成员w。
	    points.id = 0;
	//设置marker类型到 POINTS, LINE_STRIP 和 LINE_LIST
    	points.type = visualization_msgs::Marker::POINTS;
	// scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高，然而LINE_STRIP和LINE_LIST marker仅仅使用x，定义为线的宽度。单位是米。
	    points.scale.x = 0.2;
	    points.scale.y = 0.2;

	// 点为绿色
	    points.color.r = 1.0;
	    points.color.g = 1.0;
	    points.color.b = 1.0;
	    points.color.a = 1.0;

		points.text = "radar_data->objs[i].id";

    
    for(int i=0 ; i<len ;i++){
        fout_radar << id2++ <<" "<< tv.tv_sec << "." << tv.tv_usec <<" "<<radar_data->objs[i].ID<<" "<< radar_data->objs[i].m_Range <<" "<< radar_data->objs[i].m_Angle<<" "<< radar_data->objs[i].m_Rate<<endl;

	//使用正弦和余弦生成螺旋结构。POINTS和LINE_STRIP markers都仅仅需要1个点作为每个顶点，然而LINE_LIST marker需要2个点 。
	   
	      //float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
	      //float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

		//x = r * cos(angle * pi /180);
		//y = r * sin(angle * pi /180);

		 float x = radar_data->objs[i].m_Range * cos(radar_data->objs[i].m_Angle * PI / 180);

		 float y = -radar_data->objs[i].m_Range * sin(radar_data->objs[i].m_Angle * PI / 180);
		 float z = 0;
		 //cout<<id2<<"x:"<<x<<"y:"<<y<<endl;

		 geometry_msgs::Point p;
		 p.x = x;
		 p.y = y;
		 p.z = z;

		 points.points.push_back(p);

    }

 	   //发布各个markers
	    marker_pub.publish(points);
	
	    r.sleep();

	    f += 0.04;
}


int main(int argc, char **argv)
{
    std::cout << "radar"<<std::endl;
    ros::init(argc, argv, "bag_to_txt");

    //ros::init(argc, argv, "points_and_lines");
    


    ros::NodeHandle nh;
    ros::Subscriber sub_lidar=nh.subscribe("/lidar_detect_topic",1, chatterCallback_lidar); 
    ros::Subscriber sub_radar=nh.subscribe("/wave_radar_detect",1, chatterCallback_radar); 
    fout_lidar << "帧ID" <<" "<< "时间戳" <<" "<<"位置x"<<" "<< "位置y"<<" "<< "长"<<" "<<  "宽" <<" "<< "高"<<endl;
    fout_radar << "帧ID" <<" "<< "时间戳" <<" "<<"目标ID"<<" "<< "距离" <<" "<< "角度"<<" "<< "速度"<<endl;
	marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
	ros::spin();


    return 0;
}