#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <compute/map.h>
#include <custom_msgs/LaneLine.h>
#include <custom_msgs/Path.h>
class BoundingBoxCalculator{

public:
struct BoundingBox
{
  BoundingBox(){}
  cv::Point3f corners[8];/**< 8 corners of boundingbox */
  cv::Point3f size;/**< x:length, y:width, z:height, notice that length >= width*/
  cv::Point3f center; /**<box geometry center*/
//  cv::Point3f anchor;/**< the nearest corner point of box to the lidar*/
  float angle; /**< 2D (x-y plane) box main-direction(parallel to the longer edge of box) angle relative to x-axis of lidar coordinate*/
};


void draw_box(const BoundingBox& box, const int& marker_id, visualization_msgs::Marker& marker, float scale);
void draw_text(const cv::Point3f& pos, const std::string& info, const int& marker_id, 
visualization_msgs::Marker& marker, const std_msgs::ColorRGBA &color);
// void draw_lane(const std::vector<LanePoint> lane_points, const int& marker_id, 
// visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &color);
void draw_lane(const custom_msgs::LaneLine lane_points, const int& marker_id, 
visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &color);
void draw_plan_lane(const custom_msgs::Path lane_points, const int& marker_id, 
visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &color);
private:

BoundingBox scaleBox(const BoundingBox &box, float scale);


};