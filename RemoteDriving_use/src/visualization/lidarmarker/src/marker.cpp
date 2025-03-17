#include "marker.h"
BoundingBoxCalculator::BoundingBox BoundingBoxCalculator::scaleBox(const BoundingBox &box, float scale)
{
    if (fabsf(scale - 1.f) < 1e-6)
        return box;

    cv::Point3f center = box.center;

    BoundingBox box_out = box;

    for (int i = 0; i < 8; ++i)
    {
        box_out.corners[i] = (box.corners[i] - center) * scale + center;
    }

    box_out.size = box.size * scale;

    //  box_out.anchor = box.corners[findBoxAnchor(box_out)];

    return box_out;
}

void BoundingBoxCalculator::draw_box(const BoundingBox &box, const int &marker_id, visualization_msgs::Marker &marker, float scale)
{

    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.header.frame_id = "map";
    std::vector<geometry_msgs::Point> cub_points;

    BoundingBox box_s = scaleBox(box, scale);

    for (int i = 0; i < 8; ++i)
    {
        geometry_msgs::Point pts;
        pts.x = box_s.corners[i].x;
        pts.y = box_s.corners[i].y;
        pts.z = box_s.corners[i].z;
        cub_points.push_back(pts);
    }

    marker.scale.x = 0.1;
    marker.color.g = 1;
    marker.color.a = 1;
    marker.lifetime = ros::Duration(0.1);

    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[0]);
    // horizontal high points for lines
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[7]);
    marker.points.push_back(cub_points[4]);
    // vertical points for lines
    marker.points.push_back(cub_points[0]);
    marker.points.push_back(cub_points[4]);
    marker.points.push_back(cub_points[1]);
    marker.points.push_back(cub_points[5]);
    marker.points.push_back(cub_points[2]);
    marker.points.push_back(cub_points[6]);
    marker.points.push_back(cub_points[3]);
    marker.points.push_back(cub_points[7]);
}


void BoundingBoxCalculator::draw_text(const cv::Point3f& pos, const std::string& info, const int& marker_id, 
visualization_msgs::Marker& marker, const std_msgs::ColorRGBA &color)
{
	//--------------标记跟踪信息----------
	marker.id = marker_id;
    // marker.header.frame_id = "map";
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos.x;
	marker.pose.position.y = pos.y;
	marker.pose.position.z = pos.z;
    marker.ns = "pos_info";
	marker.scale.x = 1;
	marker.scale.y = 1;
	marker.scale.z = 1;
    marker.color = color;
    marker.lifetime = ros::Duration(0.1);
	marker.text = info;
}

// void BoundingBoxCalculator::draw_lane(const std::vector<LanePoint> lane_points, const int& marker_id, 
// visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &color)
// {
//     marker.id = marker_id;
//     marker.type = visualization_msgs::Marker::LINE_LIST;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.header.frame_id = "map";
//     marker.color = color;
//     marker.scale.x = 0.1;
//     marker.lifetime = ros::Duration(0);
//     geometry_msgs::Point first_point;
//     first_point.x = lane_points[0].x;
//     first_point.y = lane_points[0].y;
//     first_point.z = 0;
//     marker.points.push_back(first_point);
//     for(int i = 1; i < lane_points.size(); i++)
//     {
//         geometry_msgs::Point lane_point;
//         lane_point.x = lane_points[i].x;
//         lane_point.y = lane_points[i].y;
//         lane_point.z = 0;
//         marker.points.push_back(lane_point);
//         marker.points.push_back(lane_point);
//     }
//     marker.points.push_back(first_point);
// }

void BoundingBoxCalculator::draw_lane(const custom_msgs::LaneLine lane_points, const int& marker_id, 
visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &color)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "map";
    marker.color = color;
    marker.scale.x = 0.1;
    marker.lifetime = ros::Duration(0);
    for(int i = 0; i < lane_points.x.size(); i++)
    {
        geometry_msgs::Point lane_point;
        lane_point.x = lane_points.x[i];
        lane_point.y = lane_points.y[i];
        lane_point.z = 0;
        marker.points.push_back(lane_point);
    }
}

void BoundingBoxCalculator::draw_plan_lane(const custom_msgs::Path lane_points, const int& marker_id, 
visualization_msgs::Marker &marker, const std_msgs::ColorRGBA &color)
{
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "map";
    marker.color = color;
    marker.scale.x = 0.1;
    marker.lifetime = ros::Duration(0);
    for(int i = 0; i < lane_points.x_ref.size(); i++)
    {
        geometry_msgs::Point lane_point;
        lane_point.x = lane_points.x_ref[i];
        lane_point.y = lane_points.y_ref[i];
        lane_point.z = 0;
        marker.points.push_back(lane_point);
    }
}
