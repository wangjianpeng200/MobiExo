/*****************************************************************************************
-->>encoding :    UTF-8
-->> Copyright:   重庆邮电大学智能车辆技术团队
-->> Author:      杜悦 / 杨东
-->> Date:        2019-03-25(最新修改时间)
-->> Description: 定义了坐标系之下坐标点和姿态的数据结构，及其相互转化的接口定义。

----------------------------------------------------------------------------------------
                                    -------->
    全球地理坐标系------> 地图坐标系                 车辆坐标系
                                    <--------

                                    -------->
    全球地理坐标系------> 地图坐标系                 Frenet坐标系（路径规划中实现）
                                    <--------
-----------------------------------------------------------------------------------------
(1)、WGS-84坐标系                 (2)、地图坐标系            (3)、车辆坐标系( -> 表示车辆前进方向)

           0/360(N) 		         90[y+](N)                        180[x-]
             *                           *                               *
             *                           *                               *
             *                           *                               *
             *                           *                               *
270*********************90  180[x-]******O********0[x+]     270[y-]---------------> 90[y+]
             *                           *                               *
             *                           *                               *
             *                           *                               *
             *                           *                               *
            180                        270[y-]                         0[x+]

*******************************************************************************************/

#ifndef _TRANS_COORD_H_
#define _TRANS_COORD_H_

#include <cmath>
#include <iostream>
using namespace std;


//---->>>>坐标系数据结构声明     
//---->>>>车辆姿态
//-->>笛卡尔坐标系与直接坐标系对应关系。x-->>longitude y-->>latitude yaw-->>heading
//-->>使用相同的数据结构，但是代表的意思是各自的意义。
//-->>分别定义了车辆姿态和坐标点，两种数据结构。Frenet坐标系统请参照路径规划模块。

struct VehPose
{
    double x_lon;
    double y_lat;
    float yaw_heading;
    VehPose(){}
    VehPose(double lon,double lat,float yaw):
        x_lon(lon),y_lat(lat),yaw_heading(yaw){}
};

//-->>坐标点
struct CoordPoint
{
    double x_lon;            //x-->>longitude
    double y_lat;            //y-->>latitude
    CoordPoint(){}
    CoordPoint(double x,double y):
        x_lon(x),y_lat(y){}
};

//-->>坐标点
struct FrenetPoint
{
    double s;
    double d;
};


//------------------------------------------------------------------------------------------------------->
//---->>>>坐标系转换接口声明
//------------------------------------------------------------------------------------------------------->

/**
 * @brief distance_gps 已知两点经纬度，求距离
 * @param ponit_a      GPS点A
 * @param ponit_b      GPS点b
 * @return
 */
double distance_gps(const CoordPoint &ponit_a, const CoordPoint &ponit_b);


/**
 * @brief direction_gps  已知两点经纬度，求 ponit_b 相对 ponit_a 的方向角，遵循WGS-84坐标系角度定义
 * @param ponit_a        GPS点A
 * @param ponit_b        GPS点b
 * @return
 */
double direction_gps(const CoordPoint &ponit_a, const CoordPoint &ponit_b);


/**
 * @brief trans_pose_gps2coord 将车辆姿态从 WGS-84坐标系 转化为 地图坐标系
 * @param Gps_point_base 参考点
 * @param veh_gps_pos    WGS-84坐标系下姿态
 * @param veh_coord_pos  地图坐标系下姿态
 */
void trans_pose_gps2coord(const CoordPoint &Gps_point_base,const VehPose &veh_gps_pos,VehPose &veh_coord_pos);


/**
 * @brief trans_coords_map2veh  地图坐标系 转 车辆坐标系
 * @param src_point             需要转化的地图坐标系下的坐标点
 * @param veh_coord_pos         当前车辆姿态
 * @param des_point             转化结果，对应车辆坐标系下的坐标点
 */
void trans_coords_map2veh(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point);


/**
 * @brief trans_coords_veh2map  车辆坐标系 转 地图坐标系
 * @param src_point             需要转化的车辆坐标系下的坐标点
 * @param veh_coord_pos         当前车辆姿态
 * @param des_point             转化结果，对应地图坐标系下的坐标点
 */
void trans_coords_veh2map(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point);


/**
 * @brief trans_coords_gps2veh  地球坐标系  转 车辆坐标系
 * @param src_point             地球坐标系下转换点的GPS坐标
 * @param veh_coord_pos         地球坐标系下车辆姿态
 * @param des_point             车辆坐标系下坐标点
 */
void trans_coords_gps2veh(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point);



//---------------------------------------------------------------------------------------------------------
//-->>Map <-->Frenet 接口仅在路径规划中使用，该处只作说明不作实现，请查阅路径规划相关代码
//---------------------------------------------------------------------------------------------------------

///**
// * @brief trans_coords_map2frenet
// * @param src_point
// * @param veh_coord_pos
// * @param des_point
// */
//void trans_coords_map2frenet(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point);


///**
// * @brief trans_coords_frenet2map
// * @param src_point
// * @param veh_coord_pos
// * @param des_point
// */
//void trans_coords_frenet2map(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point);


////计算车辆坐标系下的一点相对于当前车辆的方位角
//float GPSAngle(CoordPoint coordPoint, CarGPSPose currentpose );

////坐标系转换：车辆坐标系->地球坐标系
//void CoordtoGpsPoint(CarGPSPose currentpose, CoordPoint coordPoint, GpsPoint * GpsPoint);

#endif // _TRANS_COORD_H_
