#include "ros/ros.h"
#include "custom_msgs/NaviData.h"
#include "novatel_gps_imu.h"
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include "custom_msgs/Warn.h"
#include<iostream>

#define offset_dis 0.57     //后天线到houlun向hou移动0.57m  

bool warn_pub(int source,int code,ros::ServiceClient warn_client){
    custom_msgs::Warn srv;
    static int last_code;
    srv.request.source = source;
    srv.request.code   = code;
    if(last_code == code){
        return 0;
    }
    if (warn_client.call(srv)){
        last_code = code;
        return 1;
    }
    else{
        ROS_ERROR("navi : Failed to call service warn");
        return 0;
    }
}

double x2_lon = 106.671904372;
double y2_lat = 29.7763816352;
const double ER =  6370856.00;
double distance_gps(double x1_lon,double y1_lat,double x2_lon,double y2_lat)
{
    double a_lon = (x1_lon / 180) * M_PI;  //转化为弧度
    double a_lat = (y1_lat / 180) * M_PI;

    double b_lon = (x2_lon / 180) * M_PI;
    double b_lat = (y2_lat / 180) * M_PI;

    double angle = (cos(M_PI/2 - b_lat)) * (cos(M_PI/2 - a_lat)) +
                   (sin(M_PI/2 - b_lat)) * (sin(M_PI/2 - a_lat)) * (cos(b_lon - a_lon));

    double distance = ER * acos(angle);

    return  distance;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_navigation");
    ros::NodeHandle nh;

    ros::Publisher navi_data_pub = nh.advertise<custom_msgs::NaviData>("navi_msg", 1);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("raw_imu", 1);
    custom_msgs::NaviData navi_msg;
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "imu";
    tf::Quaternion q;
	
    double lon_imu;
    double lat_imu;
	
    ros::ServiceClient warn_client  = nh.serviceClient<custom_msgs::Warn>("/warn_collect_service");

    int upd_client_fd = udp_client_init(NOVATEL_IMU_PORT);
    struct timeval timeout;
    timeout.tv_sec  = 2;  //秒
    timeout.tv_usec = 0;   //微秒
    if (setsockopt(upd_client_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        perror("setsockopt timeout failed");
    }

    struct sockaddr_in server_addr;
    bzero(&server_addr,sizeof(server_addr));

    socklen_t server_addr_len = sizeof(server_addr);


    char *server_ip = (char *)"192.168.1.45";

    unsigned short server_port = 3001;
    server_addr.sin_family = AF_INET;
    inet_pton(AF_INET,server_ip,&server_addr.sin_addr);      //将点分十进制转换为整数
    server_addr.sin_port = htons(server_port);
    bzero(&(server_addr.sin_zero),8);

    char recvbuf[1024] = {0};
    bzero(recvbuf,sizeof(recvbuf));
    int recvbuf_len =sizeof(recvbuf);

    sendto(upd_client_fd,recvbuf,recvbuf_len,0,
           (struct sockaddr*) &server_addr,server_addr_len);//成功的话返回发送的字节数，发生错误返回-1

    msg_best_pose best_pose;
    msg_rawimu rawimu;
    msg_inspvax inspvax;
    ushort temo_Message_ID;

    ros::Timer timer = nh.createTimer(ros::Duration(0.1),
                                      [&navi_msg,&navi_data_pub,&best_pose,&inspvax,&rawimu,&imu_msg,&q,&imu_pub,&warn_client]
                                      (const ros::TimerEvent& evnt){
        navi_msg.head.stamp = ros::Time::now();
        navi_msg.longitude = best_pose.lon;
        navi_msg.latitude = best_pose.lat;
        navi_msg.altitude = best_pose.hgt;
        navi_msg.heading = inspvax.Azimuth;
        navi_msg.pitch = inspvax.Pitch + 1.7;
        navi_msg.roll = inspvax.Roll;
        navi_msg.pose_type = best_pose.pos_type;
        navi_msg.speed2d = 3.6*sqrt(pow(inspvax.North_Vel,2) + pow(inspvax.East_Vel,2));
        // navi_msg.INS_Status = inspvax.INS_Status;
        // navi_msg.Lat_vari = inspvax.Lat_vari;
        // navi_msg.Lon_vari = inspvax.Lon_vari;
        navi_data_pub.publish(navi_msg);
        
        //std::cout<<fixed<<setprecision(15)<<"lon2: "<<best_pose.lon-inspvax.Lon<<" lat2: "<<best_pose.lat-inspvax.Lat<<endl;

        if(best_pose.pos_type == 50 || best_pose.pos_type == 56){
            warn_pub(2,0,warn_client);
        }else{
            warn_pub(2,0x02,warn_client);
        }

        imu_msg.header.stamp = navi_msg.head.stamp;
        imu_msg.linear_acceleration.x = rawimu.X_Accel_Output * (0.2/65536)/125;
        imu_msg.linear_acceleration.y = rawimu.Y_Accel_Output * (0.2/65536)/125;
        imu_msg.linear_acceleration.z = rawimu.Z_Accel_Output * (0.2/65536)/125;

        imu_msg.angular_velocity.x = rawimu.X_Gyro_Output * (0.008/65536)/125;
        imu_msg.angular_velocity.y = rawimu.Y_Gyro_Output * (0.008/65536)/125;
        imu_msg.angular_velocity.z = rawimu.Z_Gyro_Output * (0.008/65536)/125;

        q.setRPY(navi_msg.roll,navi_msg.pitch,navi_msg.heading);
        imu_msg.orientation.x = q[0];
        imu_msg.orientation.y = q[1];
        imu_msg.orientation.z = q[2];
        imu_msg.orientation.w = q[3];
        imu_pub.publish(imu_msg);

        ros::spinOnce();
    });

    int ret = 0;
    while (ros::ok())
    {   bzero(recvbuf,sizeof(recvbuf));
        recvfrom(upd_client_fd,recvbuf,recvbuf_len,0,
                           (struct sockaddr*)&server_addr,&server_addr_len);
        //cout<<"ret : "<< ret <<endl;
        if(ret == -1){
            best_pose.pos_type = 0;
            warn_pub(2,(int)0x01,warn_client);
        }

        memcpy(&temo_Message_ID,recvbuf + 4,sizeof(temo_Message_ID));
        switch (temo_Message_ID)
        {
            case 42:
                memcpy(&best_pose,recvbuf,sizeof(best_pose));
				 if(best_pose.pos_type == 50 || best_pose.pos_type == 56){					
				 	move_gps_point( best_pose.lon,best_pose.lat,offset_dis,inspvax.Azimuth+180,lon_imu,lat_imu );
                     //std::cout<<fixed<<setprecision(15)<<"lon2: "<<best_pose.lon-lon_imu<<" lat2: "<<best_pose.lat-lat_imu<<endl;
				 	best_pose.lon = lon_imu;
				 	best_pose.lat = lat_imu;	
                    move_gps_point( best_pose.lon,best_pose.lat,0.1,inspvax.Azimuth + 90,lon_imu,lat_imu );
                    best_pose.lon = lon_imu;
				 	best_pose.lat = lat_imu;
                    //std::cout<<"!!!!!!!!!!!!!!!!!!!!distance:  "<<distance_gps(best_pose.lon,best_pose.lat,x2_lon, y2_lat)<<std::endl;	
                    //std::cout<<fixed<<setprecision(15)<<"lon2: "<<best_pose.lon-inspvax.Lon<<" lat2: "<<best_pose.lat-inspvax.Lat<<endl;			
				 }			
                break;
            case 268:
            memcpy(&rawimu,recvbuf,sizeof(rawimu));
                break;
            case 1465:
            memcpy(&inspvax,recvbuf,sizeof(inspvax));
                break;

            default:
                break;
        }
        ros::spinOnce();
    }

    return 0;
}
