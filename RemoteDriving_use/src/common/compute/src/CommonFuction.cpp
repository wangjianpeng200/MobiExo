#include <compute/CommonFuction.h>
#include <math.h>

#define PI 3.141592654
#define ER 6370856.00                                               //6371393   地球平均半径 ;
//已知经纬度获取两点间距离
double disCalculate(double lonA, double latA, double lonB, double latB)
{
    double R = ER;
    double Aj = (lonA/180)*PI;
    double Aw = (latA/180)*PI;
    double Bj = (lonB/180)*PI;
    double Bw = (latB/180)*PI;
    double res,temp;
    temp = (cos(PI/2 - Bw)) * (cos(PI/2 - Aw)) + (sin(PI/2 - Bw)) * (sin(PI/2 - Aw)) * (cos(Bj-Aj));
    res = R*acos(temp);

    return res;
}

//已知经纬度获取两点间方位，以A为参考点，A的正北方向为零度
//顺时针依次经过一、四、三、二象限到360度，返回值的单位为度
double direcCalculate(double lonA, double latA, double lonB, double latB)
{
    double Aj = (lonA/180)*PI;//转换成弧度值
    double Aw = (latA/180)*PI;
    double Bj = (lonB/180)*PI;
    double Bw = (latB/180)*PI;

    double detj = Bj - Aj;
    double y = sin(detj) * cos(Bw);
    double x = cos(Aw) * sin(Bw) - sin(Aw) * cos(Bw) * cos(detj);
    double angle = atan2(y, x);
    angle = angle*180/PI;

    if (angle < 0)
    {
        angle = angle + 360;
    }

    return angle;
}

//坐标系转换：地球坐标系->车辆坐标系
//已知两点经纬度，将其中一点转换成车辆坐标系下的x、y
void GpstoCoordPoint(CarGPSPose currentpose, GpsPoint  gpsPoint, CoordPoint * coordPoint)
{
    double len = disCalculate(currentpose.longitude, currentpose.latitude, gpsPoint.longitude, gpsPoint.latitude);
    double Angle = direcCalculate(currentpose.longitude, currentpose.latitude, gpsPoint.longitude, gpsPoint.latitude);   //路径点与当前位置点连线与真北的夹角

    coordPoint->x = len * sin( (Angle - currentpose.cpose)/180*PI );
    coordPoint->y = len * cos( (Angle - currentpose.cpose)/180*PI );
    return;
}




//在车辆坐标系下，计算一点相对当前车辆的方位角（与真北方向的夹角），返回值的单位为弧度
float GPSAngle(CoordPoint coordPoint, CarGPSPose currentpose )
{
    float coordangle;
    float  TempAzimuth;

    coordangle = atan2(coordPoint.y, coordPoint.x);   //计算转换点与x轴正方向的夹角

    if ( 0<=coordangle && coordangle<=PI/2 ) //第一象限
    {
        TempAzimuth = currentpose.cpose*PI/180 + ( PI/2 - coordangle );
    }

    if ( -PI<=coordangle && coordangle<0 ) //第三、四象限
    {
        TempAzimuth = currentpose.cpose*PI/180 + ( PI/2 + (-coordangle) );
    }

    if ( PI/2<coordangle && coordangle<=PI ) //第二象限
    {
        TempAzimuth = currentpose.cpose*PI/180 - ( coordangle  - PI/2 );
    }

    return TempAzimuth;
}

//坐标系转换：车辆坐标系->地球坐标系
//已知一点经纬度及与另一点距离和方位角，获取另一点经纬度
void CoordtoGpsPoint(CarGPSPose currentpose, CoordPoint coordPoint, GpsPoint * GpsPoint)
{
    double len; //目标点到原点A的长度
    float  Azimuth; //目标点相对于A点的地球坐标系的方位角
    double Aj  =  currentpose.longitude * PI / 180; //A点的经度
    double Aw =  currentpose.latitude * PI / 180;  //A点的纬度
    double  temp1;
    double  temp2;
    double c;

     len = sqrt( (coordPoint.x) * (coordPoint.x) + (coordPoint.y) * (coordPoint.y) );  //计算两点距离
    Azimuth =  GPSAngle(coordPoint, currentpose);      //计算目标点的方位角

    c = len/ER;                    //弧度值

    temp1 = acos( cos(PI/2 - Aw) * cos(c) + sin(PI/2 - Aw) * sin(c) * cos(Azimuth) );
    temp2 = asin( (sin(c) * sin(Azimuth)) / sin(temp1));

    GpsPoint->longitude = ( Aj + temp2) *180 / PI;          //另一点转换后的经度
    GpsPoint->latitude = ( PI/2 - temp1 ) * 180 / PI;         //另一点转换后的纬度

    return;
}

