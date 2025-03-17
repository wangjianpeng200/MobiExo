#include "ros/ros.h"
#include "custom_msgs/NaviData.h"
#include <fstream>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_txt");
    ros::NodeHandle nh;

    std::ofstream fout("/home/nvidia/Desktop/Seed_new/src/perception/localization/navigation_driver/nova_nav/log.txt",std::ios::out);

    boost::function<void(const custom_msgs::NaviData::ConstPtr &msg)> cvt_func =
            [&](const custom_msgs::NaviData::ConstPtr &msg){
        fout<<std::setprecision(12)
           <<msg->longitude<<" "
          <<msg->latitude<<" "
         <<msg->heading<<std::endl;
    };
    ros::Subscriber sub = nh.subscribe("/navi_msg", 1, cvt_func);

    ros::spin();

    return 0;
}
