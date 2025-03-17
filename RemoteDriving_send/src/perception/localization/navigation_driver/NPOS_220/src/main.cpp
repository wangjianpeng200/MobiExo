#include <iostream>
#include "serial_port_interface.h"
#include "novatel_gps_imu.h"
#include <fstream>

#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <iterator>

#include <ros/ros.h>

#include <custom_msgs/NaviData.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

using namespace std;
//Ctrl + / 注释或取消注释
//丢包原因：read读取到的字节数可能不是请求读取的字节数。
//解决方法：read后需对读取到的字节数加以判断，通过while循环使其强制读取到请求的字节数。

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NPOS_220_node");
    ros::NodeHandle nh;

    ros::Publisher navi_data_pub = nh.advertise<custom_msgs::NaviData>("navi_msg", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("raw_imu", 1);
    custom_msgs::NaviData navi_msg;
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    tf::Quaternion q;

    int fd = uart_open("/dev/ttyUSB1");
    if(fd == -1)
        exit(-1);

    fcntl(fd,F_SETFL,0);//阻塞read
    //通过fcntl()函数可以操作文件描述符，用以控制读取数据的状态。
    //fcntl(fd,F_SETFL,0)表示没有数据则阻塞，处于等待状态，直到有数据到来。
    //fcntl(fd,F_SETFL,FNDELAY)表示当端口没有数据时马上返回0。

    uart_conf_set(fd,230400,8,1,'N');

    // C99 中定义,typedef signed char int8_t; typedef short int int16_t;typedef int int32_t;
    int8_t sync_1 = 0;
    int8_t sync_2 = 0;
    int8_t sync_3 = 0;

    //解析后的数据
    msg_header    head;
    BEST_POSB_MSG best_pose;
    RAWIMU_MSG    rawimu;
    INSPVAX_MSG   inspvax;

    int8_t buff[200] =  {0};

    int num = 0;
    int rd; //期望read的字节数
    int ret;//实际read返回的字节数

    ros::Timer timer = nh.createTimer(ros::Duration(0.1),
                                      [&navi_msg,&navi_data_pub,&best_pose,&inspvax,&rawimu,&imu_msg,&q,&imu_pub]
                                      (const ros::TimerEvent& evnt){
        navi_msg.head.stamp = ros::Time::now();
        navi_msg.longitude = best_pose.lon;
        navi_msg.latitude = best_pose.lat;
        navi_msg.altitude = best_pose.hgt;
        navi_msg.heading = inspvax.Azimuth;
        navi_msg.pitch = inspvax.Pitch;
        navi_msg.roll = inspvax.Roll;
        navi_msg.pose_type = best_pose.pos_type;
        navi_msg.speed2d = 3.6*sqrt(pow(inspvax.North_Vel,2) + pow(inspvax.East_Vel,2));
        navi_data_pub.publish(navi_msg);

        imu_msg.header.stamp = navi_msg.head.stamp;
        imu_msg.linear_acceleration.x = rawimu.X_Accel_Output;
        imu_msg.linear_acceleration.y = rawimu.Y_Accel_Output;
        imu_msg.linear_acceleration.z = rawimu.Z_Accel_Output;

        imu_msg.angular_velocity.x = rawimu.X_Gyro_Output;
        imu_msg.angular_velocity.y = rawimu.Y_Gyro_Output;
        imu_msg.angular_velocity.z = rawimu.Z_Gyro_Output;

        q.setRPY(navi_msg.roll,navi_msg.pitch,navi_msg.heading);
        imu_msg.orientation.x = q[0];
        imu_msg.orientation.y = q[1];
        imu_msg.orientation.z = q[2];
        imu_msg.orientation.w = q[3];
        imu_pub.publish(imu_msg);

        ros::spinOnce();
    });

    while (ros::ok()) {
        //把调用该函数的线程挂起一段时间，单位是微秒
        //usleep(1000*1); // npos每0.01s刷新一次，线程挂起0.01s
//        read(fd,&sync_1,1);
//        cout<<setfill('0')<<setw(2)<<hex<<(int)(sync_1 & 0xff)<<" ";

        //同步消息帧，消息帧以数据流形式输出。
        rd = 0;
        while(rd<1){
        //    sync_1 = 0;
            rd = read(fd,&sync_1,1);
      //  cout<<"sync1:  "<<hex<<(int)(sync_1 &0xff)<<endl;
        }
        if((sync_1 & 0xff) != 0xaa)
        {
           num++;
         //  cout<<"-------------------------"<< dec<<num<<endl;  //检查读出的数据是否有问题
        //   cout<<"sync1:  "<<hex<<(int)(sync_1 &0xff)<<endl;
           continue;
        }
        num = 0;

        rd = 0;
        while(rd<1){
             sync_2 = 0;
            rd = read(fd,&sync_2,1);
        }
        if((sync_2 & 0xff) != 0x44)
        continue;

        rd = 0;
        while(rd<1){
             sync_3 = 0;
            rd = read(fd,&sync_3,1);
        }
        if((sync_3 & 0xff) != 0x12)
        continue;

        //cout<<"sync:  "<<hex<<(int)(sync_1 &0xff)<<"  "<<(int)(sync_2 &0xff)<<"  "<<(int)(sync_3 &0xff)<<endl;

        //-->>同步完成后读取消息头并拷贝
        buff[0] = 0xaa;
        buff[1] = 0x44;
        buff[2] = 0x12;
        //read(fd,buff + 3,sizeof(msg_header) - 3);
        rd = 25;
        while(rd > 0){
            ret = (int)read(fd,buff + 3 + 25 - rd,rd);
            if(ret == -1){
                ret = 0;
            }
            rd=rd - ret;
        }
        memcpy(&head,buff,sizeof (msg_header)); //解析
       // cout<<dec<<"message_id:  "<<head.message_id<<endl;

        switch (head.message_id){
            case 42:
                rd = 76;
                while(rd > 0){
                ret = (int)read(fd,buff + sizeof (msg_header) + 76 - rd,rd);
                if(ret == -1){
                    ret = 0;
                }
                rd=rd - ret;
                }
                //read(fd,buff + sizeof (msg_header),sizeof (BEST_POSB_MSG) - sizeof (msg_header));
                memcpy(&best_pose,buff,sizeof (BEST_POSB_MSG)); //解析
               // printf("%lf %lf\n",best_pose.lat,best_pose.lon);
              // cout<<"pos_type" <<best_pose.pos_type<<endl; //输出位置类型
               cout<<"best_pose w j"<<" "<<best_pose.lat<<" "<<best_pose.lon<<endl; //输出纬度、经度  参考值 东经106.613922,北纬29.53832
                break;
            case 268:
                rd = 44;
                while(rd > 0){
                ret = (int)read(fd,buff + sizeof (msg_header) + 44 - rd,rd);
                if(ret == -1){
                    ret = 0;
                }
                rd=rd - ret;
                }
                 cout<<"1212121212";
                //read(fd,buff + sizeof (msg_header),sizeof (RAWIMU_MSG) - sizeof (msg_header));
                memcpy(&rawimu,buff,sizeof (RAWIMU_MSG));  //解析
                break;
            case 1465:
                rd = 130;
                while(rd > 0){
                ret = (int)read(fd,buff + sizeof (msg_header) + 130 - rd,rd);
                if(ret == -1){
                    ret = 0;
                }
                rd=rd - ret;
                }
                //read(fd,buff + sizeof (msg_header),sizeof (INSPVAX_MSG)- sizeof (msg_header));
                memcpy(&inspvax,buff,sizeof (INSPVAX_MSG));  //解析
                cout<<"inspvax w j h"<<" "<<inspvax.Lat<<" "<<inspvax.Lon<<" "<<inspvax.Height<<endl; //输出纬度、经度  参考值 东经106.613922,北纬29.53832
                break;
            default:
                break;
        }

        //memset(buff+3,0,sizeof (buff)); //清空数组
        memset(buff,0,sizeof (buff)); //清空数组
        ros::spinOnce();
    }

    return 0;
}

