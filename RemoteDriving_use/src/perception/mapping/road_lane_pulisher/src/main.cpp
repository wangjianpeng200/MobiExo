#include "road_lane.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_road_lane");
    RoadLane lane;
    lane.exec();
} 
