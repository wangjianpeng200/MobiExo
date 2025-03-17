#include "imu_msg.h"
#include "uart_api.h"
#include "iostream"
#include "string"
#include "cstring"

#include <ros/ros.h>
#include <ros/time.h>
#include <custom_msgs/NaviData.h>
#include <sensor_msgs/Imu.h>
#include <thread>
#include <pthread.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "cgi_navigation_node");
    ros::NodeHandle nh;

    ros::Publisher navi_data_pub = nh.advertise<custom_msgs::NaviData>("navi_msg", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("raw_imu", 1);
    custom_msgs::NaviData navi_msg;
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    tf::Quaternion q;

    std::string recv_string;
    std::string gpg_msg;
    std::string header;
    std::string uart_num;

    int start_position = 0;
    int back_position = 0;         //记录$位置
    int comma_position_front = 0;
    int comma_position_back = 0;   //记录，位置

    int index = 0;
    ssize_t ret = 0;
    int read_num = 0;

    //用于保存解析后数据
    gpgga gpgga_msg;
    gpvtg gpvtg_msg;
    pashr pashr_msg;
    gpchc gpchc_msg;

    nh.param<std::string>("uart_num", uart_num,"/dev/ttyUSB0");

    char msg_buff[512];
    int uart = uart_open(uart_num.c_str());
    fcntl(uart,F_SETFL,0);  //阻塞
    uart_conf_set(uart,230400,8,1,'N');
    memset(msg_buff,0,512);

    ros::Timer timer = nh.createTimer(ros::Duration(0.05),
                                      [&navi_msg,&navi_data_pub,&gpgga_msg,&gpvtg_msg,&gpchc_msg,&pashr_msg,&imu_msg,&q,&imu_pub]
                                      (const ros::TimerEvent& evnt){
        navi_msg.head.stamp = ros::Time::now();
        navi_msg.longitude = gpchc_msg.Longitude;
        navi_msg.latitude = gpchc_msg.Lattitude;
        navi_msg.altitude = gpchc_msg.Altitude;
        navi_msg.heading = gpchc_msg.Heading;
        navi_msg.pitch = gpchc_msg.Pitch;
        navi_msg.roll = gpchc_msg.Roll;
        navi_msg.pose_type = gpchc_msg.Status;
        navi_msg.speed2d = gpchc_msg.V*3.6;
        navi_data_pub.publish(navi_msg);

        imu_msg.header.stamp = navi_msg.head.stamp;
        imu_msg.linear_acceleration.x = gpchc_msg.acc_x;
        imu_msg.linear_acceleration.y = gpchc_msg.acc_y;
        imu_msg.linear_acceleration.z = gpchc_msg.acc_z;

        imu_msg.angular_velocity.x = gpchc_msg.gyro_x;
        imu_msg.angular_velocity.y = gpchc_msg.gyro_y;
        imu_msg.angular_velocity.z = gpchc_msg.gyro_z;

        q.setRPY(navi_msg.roll,navi_msg.pitch,navi_msg.heading);
        imu_msg.orientation.x = q[0];
        imu_msg.orientation.y = q[1];
        imu_msg.orientation.z = q[2];
        imu_msg.orientation.w = q[3];
        imu_pub.publish(imu_msg);

        ros::spinOnce();
    });

    ros::Rate rate(10);
    while (ros::ok()){
        for (read_num=0 ; read_num<500; read_num+=ret) { //强制读取500字符
	 	    ret = read(uart,msg_buff+read_num,500-read_num);
	    }
        recv_string = msg_buff;

        start_position = 0;
        back_position = 0;
     
        while(ros::ok()){
            if((start_position = recv_string.find("$",back_position)) == std::string::npos)
                break;
            if((back_position = recv_string.find("$",start_position+1)) == std::string::npos)
                break;
            gpg_msg = recv_string.substr(start_position+1,back_position-start_position-2);//截取$之间的字符串
            header = recv_string.substr(start_position+1,5);

            //std::cout<< gpg_msg <<std::endl;

            comma_position_front = 0;
            comma_position_back = 0;
            index = 0;

            //解析GPCHC消息
            if(!strncmp(header.c_str(),"GPCHC",5)){

                strcpy(gpvtg_msg.header,header.c_str());

                while(ros::ok()){
                    if((comma_position_front = gpg_msg.find(",",comma_position_back)) == std::string::npos)
                        break;
                    if((comma_position_back = gpg_msg.find(",",comma_position_front+1)) == std::string::npos)
                        break;
                    
                    index++;

                    switch (index){ 
                    case 1:
                        gpchc_msg.GPSWeek = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 2:
                        gpchc_msg.GPSTime = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 3:
                        gpchc_msg.Heading = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 4:
                        gpchc_msg.Pitch = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 5:
                        gpchc_msg.Roll = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 6:
                        gpchc_msg.gyro_x = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 7:
                        gpchc_msg.gyro_y = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 8:
                        gpchc_msg.gyro_z = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 9:
                        gpchc_msg.acc_x = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 10:
                        gpchc_msg.acc_y = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 11:
                        gpchc_msg.acc_z = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 12:
                        gpchc_msg.Lattitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 13:
                        gpchc_msg.Longitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 14:
                        gpchc_msg.Altitude = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 15:
                        gpchc_msg.Ve = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 16:
                        gpchc_msg.Vn = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 17:
                        gpchc_msg.Vu = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 18:
                        gpchc_msg.V = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 19:
                        gpchc_msg.NSV1 = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 20:
                        gpchc_msg.NSV2 = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 21:
                        gpchc_msg.Status = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    case 22:
                        gpchc_msg.Age = atof(gpg_msg.substr(comma_position_front+1,comma_position_back-comma_position_front-1).c_str());
                        break;
                    default: 
                        break;    
                    }     
                }

                 ///std::cout<< "gpvtg_msg.Speed_over_ground_km:" << gpvtg_msg.Speed_over_ground_km <<std::endl;
            }
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        ros::spinOnce();
        // rate.sleep();
    }
    
    return 0;
}
