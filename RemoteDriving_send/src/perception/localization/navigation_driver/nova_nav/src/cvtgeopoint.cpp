#include "ros/ros.h"
#include "std_msgs/String.h"

#include <fstream>

#include <custom_msgs/NaviData.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_cvtdata");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geographic_msgs::GeoPointStamped>("/gps/geopoint",1);
    ros::Publisher lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points",1);
    geographic_msgs::GeoPointStamped geo_point;

    std::ofstream fout;
    boost::function<void(const custom_msgs::NaviData::ConstPtr &msg)> cvt_func =
            [&](const custom_msgs::NaviData::ConstPtr &msg){
        geo_point.header = msg->head;
        geo_point.position.latitude =  msg->latitude;
        geo_point.position.longitude =  msg->longitude;
        geo_point.position.altitude =  msg->altitude;
        pub.publish(geo_point);
        ros::spinOnce();
    };
    ros::Subscriber sub = nh.subscribe("navi_msg", 1, cvt_func);

    ros::spin();

    return 0;
}
