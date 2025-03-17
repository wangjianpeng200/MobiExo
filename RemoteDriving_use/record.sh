#! /bin/zsh
#sudo ptpd -M -i eno1 -C
source devel/setup.zsh
rosbag record /rs128_top/rslidar_points /navi_msg /road_lane

