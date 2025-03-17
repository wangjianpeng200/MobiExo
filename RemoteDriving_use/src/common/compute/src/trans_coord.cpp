#include "trans_coord.h"

//---->>>>常量定义
const double ER =  6370856.00;      //地球平均半径 ER = 6371393


//---->>>>接口函数定义

double distance_gps(const CoordPoint &ponit_a, const CoordPoint &ponit_b)
{
    double a_lon = (ponit_a.x_lon / 180) * M_PI;  //转化为弧度
    double a_lat = (ponit_a.y_lat / 180) * M_PI;

    double b_lon = (ponit_b.x_lon / 180) * M_PI;
    double b_lat = (ponit_b.y_lat / 180) * M_PI;

    double angle = (cos(M_PI_2 - b_lat)) * (cos(M_PI_2 - a_lat)) +
                   (sin(M_PI_2 - b_lat)) * (sin(M_PI_2 - a_lat)) * (cos(b_lon - a_lon));

    double distance = ER * acos(angle);

    return  distance;
}


double direction_gps(const CoordPoint &ponit_a, const CoordPoint &ponit_b)
{
    double a_lon = (ponit_a.x_lon / 180) * M_PI;  //转化为弧度
    double a_lat = (ponit_a.y_lat / 180) * M_PI;

    double b_lon = (ponit_b.x_lon / 180) * M_PI;
    double b_lat = (ponit_b.y_lat / 180) * M_PI;

    double det_lon = b_lon - a_lon;
    double y = sin(det_lon) * cos(b_lat);
    double x = cos(a_lat) * sin(b_lat) - sin(a_lat) * cos(b_lat) * cos(det_lon);

    double angle = atan2(y, x);
    angle = angle * 180 / M_PI;   //转化为度

    angle = angle < 0 ? angle + 360 : angle;

    return angle;
}

void trans_pose_gps2coord(const CoordPoint &Gps_point_base,const VehPose &veh_gps_pos,VehPose &veh_coord_pos)
{
    const CoordPoint veh_gps_point(veh_gps_pos.x_lon,veh_gps_pos.y_lat);

    double point_dis = distance_gps(Gps_point_base,veh_gps_point);                  //-->>两点gps的间距离
    double angle = direction_gps(Gps_point_base,veh_gps_point);                     //-->>相对真北方向夹角

    angle = ((angle >= 0) && (angle < 90)) ? 90 - angle : 450 - angle;                //-->>真北方向夹角转地图坐标系

    angle = angle *M_PI /180;
    veh_coord_pos.x_lon = point_dis * cos(angle);                                       //-->>x轴分量
    veh_coord_pos.y_lat = point_dis * sin(angle);                                       //-->>y轴分量
    veh_coord_pos.yaw_heading = (veh_gps_pos.yaw_heading >= 0) && (veh_gps_pos.yaw_heading < 90) ?   //-->>真北方向夹角转地图坐标系
                        90 - veh_gps_pos.yaw_heading : 450 - veh_gps_pos.yaw_heading;
    return ;
}


void trans_coords_map2veh(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point)
{
    double deta_x = src_point.x_lon - veh_coord_pos.x_lon;
    double deta_y = src_point.x_lon - veh_coord_pos.x_lon;
    double deta_angle = (veh_coord_pos.yaw_heading / 180) * M_PI;

    des_point.x_lon = deta_x * cos(deta_angle) + deta_y * sin(deta_angle);
    des_point.y_lat = deta_y * cos(deta_angle) - deta_x * sin(deta_angle);

    return;
}

void trans_coords_veh2map(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point)
{
    double deta_x = veh_coord_pos.x_lon;
    double deta_y = veh_coord_pos.y_lat;
    double deta_angle = - (veh_coord_pos.yaw_heading / 180) * M_PI;//veh_coord_pos.yaw_heading

    des_point.x_lon = src_point.x_lon * cos(deta_angle) + src_point.y_lat * sin(deta_angle) + deta_x;
    des_point.y_lat = src_point.y_lat * cos(deta_angle) - src_point.x_lon * sin(deta_angle) + deta_y;

    return;
}


//void trans_coords_map2frenet(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point){}

//void trans_coords_frenet2map(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point){}


void trans_coords_gps2veh(const CoordPoint &src_point,const VehPose &veh_coord_pos,CoordPoint &des_point)
{
    const CoordPoint veh_gps_point = {veh_coord_pos.x_lon,veh_coord_pos.y_lat};
    double len = distance_gps(src_point, veh_gps_point);
    double temp_angle = direction_gps(veh_gps_point ,src_point);  //路径点与当前位置点连线与真北的夹角

    des_point.x_lon = len * sin( (temp_angle - veh_coord_pos.yaw_heading) / 180 * M_PI );
    des_point.y_lat = len * cos( (temp_angle - veh_coord_pos.yaw_heading) / 180 * M_PI );
    return;
}


////在车辆坐标系下，计算一点相对当前车辆的方位角（与真北方向的夹角），返回值的单位为弧度
//float GPSAngle(CoordPoint coordPoint, CarGPSPose currentpose )
//{
//    float coordangle;
//    float  TempAzimuth;

//    coordangle = atan2(coordPoint.y, coordPoint.x);   //计算转换点与x轴正方向的夹角

//    if ( 0<=coordangle && coordangle<=PI/2 ) //第一象限
//    {
//        TempAzimuth = currentpose.cpose*PI/180 + ( PI/2 - coordangle );
//    }

//    if ( -PI<=coordangle && coordangle<0 ) //第三、四象限
//    {
//        TempAzimuth = currentpose.cpose*PI/180 + ( PI/2 + (-coordangle) );
//    }

//    if ( PI/2<coordangle && coordangle<=PI ) //第二象限
//    {
//        TempAzimuth = currentpose.cpose*PI/180 - ( coordangle  - PI/2 );
//    }

//    return TempAzimuth;
//}

////坐标系转换：车辆坐标系->地球坐标系
////已知一点经纬度及与另一点距离和方位角，获取另一点经纬度
//void CoordtoGpsPoint(CarGPSPose currentpose, CoordPoint coordPoint, GpsPoint * GpsPoint)
//{
//    double len; //目标点到原点A的长度
//    float  Azimuth; //目标点相对于A点的地球坐标系的方位角
//    double Aj  =  currentpose.longitude * PI / 180; //A点的经度
//    double Aw =  currentpose.latitude * PI / 180;  //A点的纬度
//    double  temp1;
//    double  temp2;
//    double c;

//     len = sqrt( (coordPoint.x) * (coordPoint.x) + (coordPoint.y) * (coordPoint.y) );  //计算两点距离
//    Azimuth =  GPSAngle(coordPoint, currentpose);      //计算目标点的方位角

//    c = len/ER;                    //弧度值

//    temp1 = acos( cos(PI/2 - Aw) * cos(c) + sin(PI/2 - Aw) * sin(c) * cos(Azimuth) );
//    temp2 = asin( (sin(c) * sin(Azimuth)) / sin(temp1));

//    GpsPoint->longitude = ( Aj + temp2) *180 / PI;          //另一点转换后的经度
//    GpsPoint->latitude = ( PI/2 - temp1 ) * 180 / PI;         //另一点转换后的纬度

//    return;
//}

