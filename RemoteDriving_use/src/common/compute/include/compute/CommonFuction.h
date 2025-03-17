/*************************************************
    文件名：CommonFuction.h
    说明：坐标转换公共函数，包括地球坐标系到车辆坐标系，车辆
        坐标系到地球坐标系
    作者：杜悦
    时间：2018.4

*************************************************/
#ifndef COMMONFUCTION_H
#define COMMONFUCTION_H

#include <iostream>
#include <math.h>

typedef struct current_car_gps_pose  //车辆的当前位姿
{
    double longitude;
    double latitude;
    float cpose;    //航向角

}CarGPSPose;

typedef struct coord_point    //路径的车辆参考系坐标
{
    double x;
    double y;

}CoordPoint;

typedef  struct gps_point_path   //路径点的GPS坐标
{
    double longitude;
    double latitude;

}GpsPoint;


//已知经纬度获取两点间距离
double disCalculate(double lonA, double latA, double lonB, double latB);

//已知经纬度获取两点间方位，以A为参考点，A的正北方向为零度
//顺时针依次经过一、四、三、二象限到360度，返回值的单位是度
double direcCalculate(double lonA, double latA, double lonB, double latB);

//计算车辆坐标系下的一点相对于当前车辆的方位角
float GPSAngle(CoordPoint coordPoint, CarGPSPose currentpose );


//坐标系转换：地球坐标系->车辆坐标系
void GpstoCoordPoint(CarGPSPose currentpose, GpsPoint  gpsPoint, CoordPoint * coordPoint);

//坐标系转换：车辆坐标系->地球坐标系
void CoordtoGpsPoint(CarGPSPose currentpose, CoordPoint coordPoint, GpsPoint * GpsPoint);

#endif // COMMONFUCTION_H
