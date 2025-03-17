#include <ros/ros.h>
#include "marker.h"
#include <custom_msgs/Object.h>
#include <custom_msgs/ObjectArray.h>
#include <string.h>
#include <custom_msgs/LidarRawObject.h>
#include <custom_msgs/LidarRawObjectArray.h>
#include <custom_msgs/LaneLineArray.h>
ros::Publisher ass_maker_pub, pub_lane_marker, obj_marker_pub, ukf_maker_pub, plan_lane_marker_pub;
std_msgs::ColorRGBA blue, brown, white;
BoundingBoxCalculator lane,boxer;
void ass_callback(const custom_msgs::ObjectArray &ass_data)
{
    BoundingBoxCalculator AssMarker;
    visualization_msgs::MarkerArray assmarkers;
    cv::Point3f ass_center;
    for(int i = 0; i < ass_data.objs.size(); i++)
    {
        ass_center.x = (ass_data.objs[i].bbox_point[3].x +  ass_data.objs[i].bbox_point[5].x)/2;
        ass_center.y = (ass_data.objs[i].bbox_point[3].y +  ass_data.objs[i].bbox_point[5].y)/2;
        ass_center.z = (ass_data.objs[i].bbox_point[3].z +  ass_data.objs[i].bbox_point[5].z)/2 + 2.0;
        visualization_msgs::Marker assmarker;
        assmarker.header.frame_id = ass_data.header.frame_id;
       // AssMarker.draw_text(ass_center, std::to_string(ass_data.objs[i].id), i, assmarker, white);
        //AssMarker.draw_text(ass_center, "v:" + std::to_string(ass_data.objs[i].v).substr(0,3), i, assmarker, white);
        AssMarker.draw_text(ass_center, "h:" + std::to_string(ass_data.objs[i].lwh.z).substr(0,3), i, assmarker, white);
       // AssMarker.draw_text(ass_center, "s:" + std::to_string(ass_data.objs[i].s_pos).substr(0,3), i, assmarker, white);
        assmarkers.markers.push_back(assmarker);
        //ass_center.z += 2.0;
       // AssMarker.draw_text(ass_center, "vx" + std::to_string(ass_data.objs[i].vx), i + ass_data.objs.size(), assmarker, blue);
        // assmarkers.markers.push_back(assmarker);
    }
    ass_maker_pub.publish(assmarkers);
}

void ukf_callback(const custom_msgs::ObjectArray &ukf_data)
{
    BoundingBoxCalculator UKFMarker;
    visualization_msgs::MarkerArray ukfmarkers;
    cv::Point3f ukf_center;
    for(int i = 0; i < ukf_data.objs.size(); i++)
    {
        ukf_center.x = (ukf_data.objs[i].bbox_point[3].x +  ukf_data.objs[i].bbox_point[5].x)/2;
        ukf_center.y = (ukf_data.objs[i].bbox_point[3].y +  ukf_data.objs[i].bbox_point[5].y)/2;
        ukf_center.z = (ukf_data.objs[i].bbox_point[3].z +  ukf_data.objs[i].bbox_point[5].z)/2 + 4.0;
        visualization_msgs::Marker ukfmarker;
        ukfmarker.header.frame_id = ukf_data.header.frame_id;
        UKFMarker.draw_text(ukf_center, std::to_string(ukf_data.objs[i].id), i, ukfmarker, blue);
        // UKFMarker.draw_text(ukf_center, "vy:" + std::to_string(ukf_data.objs[i].vy), i, ukfmarker, blue);
        ukfmarkers.markers.push_back(ukfmarker);
        // ass_center.z += 2.0;
        // AssMarker.draw_text(ass_center, "vy" + std::to_string(ass_data.objs[i].vy), i + ass_data.objs.size(), assmarker, blue);
        // assmarkers.markers.push_back(assmarker);
    }
    ukf_maker_pub.publish(ukfmarkers);
}

void obj_callback(const custom_msgs::LidarRawObjectArray &detect_obj)
{
  visualization_msgs::MarkerArray obj_markers;
  obj_markers.markers.clear();
  int box_count = 0;
  for (int i = 0; i < detect_obj.objs.size(); ++i)
  {
    visualization_msgs::Marker obj_box_marker;
    obj_box_marker.header.frame_id = detect_obj.head.frame_id;
    BoundingBoxCalculator::BoundingBox obj_box;
    for(int j = 0; j < 8 ; j++)
    {
      obj_box.corners[j].x = detect_obj.objs[i].bbox_point[j].x;
      obj_box.corners[j].y = detect_obj.objs[i].bbox_point[j].y;
      obj_box.corners[j].z = detect_obj.objs[i].bbox_point[j].z;
    }
    obj_box.size.x = detect_obj.objs[i].lwh.x;
    obj_box.size.y = detect_obj.objs[i].lwh.y;
    obj_box.size.z = detect_obj.objs[i].lwh.z;
    obj_box.center.x = detect_obj.objs[i].x_pos;
    obj_box.center.y = detect_obj.objs[i].y_pos;
    obj_box.center.z = detect_obj.objs[i].z_pos;
    boxer.draw_box(obj_box, box_count++, obj_box_marker, 1.1);
    cv::Point3f pos;
    pos.x = obj_box.center.x;
    pos.y = obj_box.center.y;
    pos.z = obj_box.center.z + 1.7;
    std::string info;
    float area = obj_box.size.x * obj_box.size.y;
    float hight = obj_box.size.z;
    if (area > 3 && area < 9 && hight < 2)
      info = "Car";
    else if (area >= 9)
      info = "Unkown";
    else if (area > 0.1 && area <= 1.5 && hight < 1.5 && hight > 0.5 && obj_box.center.z < 0.9)
      info = "Pedestrian";
    else
      info = "Unkown";
    visualization_msgs::Marker obj_txt_marker;
    obj_txt_marker.header.frame_id = detect_obj.head.frame_id;
    boxer.draw_text(pos, info, box_count++, obj_txt_marker, brown);
    obj_markers.markers.push_back(obj_box_marker);
    // obj_markers.markers.push_back(obj_txt_marker);
    }
    // pub_marker.publish(markers);
    obj_marker_pub.publish(obj_markers);
}

void lane_callback(const custom_msgs::LaneLineArray &lane_msg)
{
  visualization_msgs::MarkerArray marker_lane;
  int lane_count = 0;
  for(int i = 0; i < lane_msg.lines.size(); i++)
  {
    if(lane_msg.lines[i].type == 0) 
    {
      visualization_msgs::Marker road_lane_marker;
      lane.draw_lane(lane_msg.lines[i], lane_count++, road_lane_marker, blue);
      marker_lane.markers.push_back(road_lane_marker);
    }
    if(lane_msg.lines[i].type == 1) 
    {
      visualization_msgs::Marker road_lane_marker;
      lane.draw_lane(lane_msg.lines[i], lane_count++, road_lane_marker, white);
      marker_lane.markers.push_back(road_lane_marker);
    }
  }
  pub_lane_marker.publish(marker_lane);
}

void plan_lane_callback(const custom_msgs::Path &msg)
{
  visualization_msgs::MarkerArray MARKER;
  visualization_msgs::Marker plan_lane_marker;
  lane.draw_plan_lane(msg, 1, plan_lane_marker, white);
  MARKER.markers.push_back(plan_lane_marker);
  plan_lane_marker_pub.publish(MARKER);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lidar_marker");
  ros::NodeHandle nh("~");
  blue.a = 1;
  blue.r = 0;
  blue.g = 0;
  blue.b = 255.0/255.0;
  brown.a = 1;
  brown.r = 1;
  brown.g = 0.5;
  brown.b = 0.f;
  white.a = 1;
  white.r = 255.0/255;
  white.g = 251.0/255;
  white.b = 240.0/255;

  std::string road_lane_topic;
  nh.param<std::string>("road_lane_topic", road_lane_topic, "/road_lane");
  ros::Subscriber lane_sub = nh.subscribe(road_lane_topic, 1, lane_callback);

  std::string ass_object_topic;
  nh.param<std::string>("ass_object_topic", ass_object_topic, "/ass_object");
  ros::Subscriber ass_sub = nh.subscribe(ass_object_topic, 1, ass_callback); //订阅关联目标

  std::string ukf_object_topic;
  nh.param<std::string>("ukf_object_topic", ukf_object_topic, "/ObjectArray");
  ros::Subscriber ukf_sub = nh.subscribe(ukf_object_topic, 1, ukf_callback);
  
  std::string detect_object_topic;
  nh.param<std::string>("detect_object_topic", detect_object_topic, "/detect_topic");
  ros::Subscriber filter_obj_sub = nh.subscribe(detect_object_topic, 1, obj_callback);

  std::string path_topic;
  nh.param<std::string>("path_topic", path_topic, "/marker_path");
  ros::Subscriber plan_lane_sub = nh.subscribe(path_topic, 1, plan_lane_callback);

  pub_lane_marker = nh.advertise<visualization_msgs::MarkerArray>("/lane_marker", 1, true);
  ass_maker_pub = nh.advertise<visualization_msgs::MarkerArray>("/ass_marker", 1, true);
  ukf_maker_pub = nh.advertise<visualization_msgs::MarkerArray>("/ukf_marker", 1, true);
  obj_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/object_marker", 1, true);
  plan_lane_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/plan_lane_marker",1,true);
  ros::spin();
  return 0;
}