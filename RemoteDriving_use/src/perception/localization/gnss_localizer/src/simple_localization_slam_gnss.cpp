#include "ros/ros.h"
#include <custom_msgs/UWBPose.h>
#include <custom_msgs/NaviData.h>
#include <custom_msgs/SlamPose.h> //slam
#include <compute/trans_coord.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <fstream>
#include <cstdlib>
#include <std_msgs/Bool.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h" 
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_listener.h>

tf2_ros::Buffer *tf_buffer_ptr;
tf2_ros::TransformListener *tf_listener_ptr; 

/*
使用前，最好去检查urdf的连接关系，矫正 base_link、 imu、 center_back的tf关系！！！
*/

class SimpleLocalization
{

public:
    SimpleLocalization();

public:
    void exec();

private:
    ros::NodeHandle nh;
    ros::Publisher chatter_pub;
    ros::Publisher speed_pub;
    ros::Publisher slam_pose_pub;
    ros::Subscriber sub;
 
    ros::Subscriber slam_positioning_sub; //slam
    tf::TransformBroadcaster br;
    // tf2_ros::StaticTransformBroadcaster static_br; 

    ros::Publisher path_pub;
    visualization_msgs::Marker path_marker; 

    double map_longitude;
    double map_latitude;
    float map_yaw;
    float theta;
    VehPose pos_map;
    CoordPoint map_ori;

    geometry_msgs::Pose2D slam_pose;
    int Is_slam_start;
    bool slam_origin_get_once;
    geometry_msgs::Pose2D cur_pose;

    tf::TransformListener listener;
	tf::StampedTransform transform;
    Eigen::Matrix4f     Tgo_so; //slam在gnss下的变换关系，so: slam_origin， go: gnss_origin
    Eigen::Vector3f euler_angles; //slam的航向角

private: 
    void onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg);
    void getPoseonMap(const custom_msgs::NaviData::ConstPtr &msg);
    void onSlamMsgRecvd(const geometry_msgs::PoseStamped::ConstPtr &msg);
    bool TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix);    // ROS TF TO Eigen 

private:
    Eigen::AngleAxisd rollAngle;
    Eigen::AngleAxisd pitchAngle;
    Eigen::AngleAxisd yawAngle;
    Eigen::Quaterniond quaternion; //四元数，用于两个坐标系之间的转换
    Eigen::Quaterniond quaternion2; 
    std_msgs::ColorRGBA white;
    std_msgs::ColorRGBA lawngreen;
    std_msgs::ColorRGBA cyan;

    bool is_display_trajectory; //是否显示轨迹
    bool is_save_trajectory;    //是否保存轨迹
    std::fstream fout1,fout2,fout3;
    geometry_msgs::TransformStamped tfs; 
    tf2::Quaternion qtn;
    float aftYaw;

    geometry_msgs::TransformStamped transformStamped;   
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_localization");

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    tf_buffer_ptr = &tf_buffer;
    tf_listener_ptr = &tf_listener; 

    SimpleLocalization loc;
    loc.exec(); 
    return 0;
}

SimpleLocalization::SimpleLocalization() 
{
    rollAngle = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    pitchAngle = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY());
    if (ros::param::has("~map_longitude"))
        ros::param::get("~map_longitude", map_longitude);
    else
    {
        ROS_WARN("ros param map_longitude not found, the localoization node will not work");
    }
    if (ros::param::has("~map_latitude"))
        ros::param::get("~map_latitude", map_latitude);
    else
    {
        ROS_WARN("ros param map_latitude not found, the localoization node will not work");
    }
    if (ros::param::has("~map_yaw"))
        ros::param::get("~map_yaw", map_yaw);
    else
    {
        ROS_WARN("ros param map_latitude not found, the localoization node will not work");
    }
    if ( !ros::param::has("/Is_slam_start") )        //slam是否开始启动的标志
    {
        ROS_WARN("ros param Is_start_slam not found, the localoization node will not work");
    }
    if ( !ros::param::has("/slam_origin_get_once") )   
    {
        ROS_WARN("ros param slam_origin_get_once not found, the localoization node will not work");
    }
    slam_origin_get_once = false; 
    Is_slam_start = 0;  

    map_ori.x_lon = map_longitude;    
    map_ori.y_lat = map_latitude;    

    slam_pose_pub = nh.advertise<geometry_msgs::Pose2D>("slam_pose", 1);
    chatter_pub = nh.advertise<geometry_msgs::Pose2D>("cur_pose", 1);
    speed_pub = nh.advertise<std_msgs::Float32>("cur_speed", 1);

    sub = nh.subscribe("navi_msg", 1, &SimpleLocalization::onNaviMsgRecvd, this);                   //惯导定位发布
    slam_positioning_sub = nh.subscribe("tracked_pose", 1, &SimpleLocalization::onSlamMsgRecvd, this);      //slam定位发布

    nh.param<bool>("/gnss_localizer_node/is_display_trajectory", is_display_trajectory, false);
    nh.param<bool>("/gnss_localizer_node/is_save_trajectory", is_save_trajectory, false);

    if (is_save_trajectory)
    { //打开轨迹记录
        const char *env_p = std::getenv("SEED_HOME");  
        if (env_p == NULL)
        {
            ROS_FATAL("exception happened while reading env variable \"SEED_HOME\" "); 
            exit(1);
        }
        std::string home_path = env_p;
        fout1.open(home_path + "/data/navigation/pureGnssTrajectory.txt", std::ios::out); // 惯导的定位点
        fout2.open(home_path + "/data/navigation/pureSlamTrajectory.txt", std::ios::out); // slam的定位点
        fout3.open(home_path + "/data/navigation/trajectory.txt", std::ios::out);         // 室内外混合轨迹，都是在map下的点
    }

    if (is_display_trajectory) 
    { 
        white.a = 1;
        white.r = 1;
        white.g = 1;
        white.b = 1;

        lawngreen.a = 1;
        lawngreen.r = 124.0 / 255;
        lawngreen.g = 252.0 / 255;
        lawngreen.b = 1;

        cyan.a = 1;
        cyan.r = 0;
        cyan.g = 1;
        cyan.b = 1;
        path_pub = nh.advertise<visualization_msgs::Marker>("trajectory", 1);
        path_marker.ns = "trajectory";
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.header.frame_id = "map";
        path_marker.scale.x = 0.1;
        path_marker.color = white;
        path_marker.lifetime = ros::Duration(0);

    }
}

void SimpleLocalization::exec()
{
    ros::spin();
}

//slam
void SimpleLocalization::onSlamMsgRecvd(const geometry_msgs::PoseStamped::ConstPtr &msg) 
{
    geometry_msgs::TransformStamped  slamTrans; 
    Eigen::Matrix4f Tso_car, Tgo_car;
    Eigen::Matrix3f rotationMatrix;

    ros::param::get("/Is_slam_start", Is_slam_start);  
    ros::param::get("/slam_origin_get_once", slam_origin_get_once); 
    if(Is_slam_start!=3 || slam_origin_get_once!=true)      //不满足 return
        return; 
    
    slamTrans.transform.translation.x = msg->pose.position.x;
    slamTrans.transform.translation.y = msg->pose.position.y;
    slamTrans.transform.translation.z = msg->pose.position.z;
    slamTrans.transform.rotation = msg->pose.orientation;
    
    //convert   position  and angle
    TransformToMatrix(slamTrans, Tso_car); 
    Tgo_car = Tgo_so * Tso_car; 
    rotationMatrix = Tgo_car.block<3, 3>(0, 0);
    euler_angles = rotationMatrix.eulerAngles( 2,1,0 );	// ZYX顺序，即yaw pitch roll顺序  取出z轴偏航角即可
    // std::cout << "原始 euler_angles(0): " << (euler_angles(0) * 180 / M_PI) << std::endl;
    // std::cout << "原始 euler_angles(1): " << (euler_angles(1) * 180 / M_PI) << std::endl;
    // std::cout << "原始 euler_angles(2): " << (euler_angles(2) * 180 / M_PI) << std::endl;

    /*****************get slam pose on map*******************************/
    euler_angles(0) = euler_angles(2)!=0 ? (M_PI - euler_angles(0)) : (2*M_PI - euler_angles(0));   //heading
    aftYaw = (euler_angles(0) >= 0 && euler_angles(0) < M_PI/2) ?  (M_PI/2 - euler_angles(0)) : (2.5*M_PI - euler_angles(0));  //cur_pose

    // 2D位姿 
    slam_pose.x = Tgo_car(0, 3); 
    slam_pose.y = Tgo_car(1, 3); 
    slam_pose.theta = aftYaw * 180 / M_PI;  
    // std::cout << "转换后 euler_angles(0): " << slam_pose.theta << std::endl; 

    slam_pose_pub.publish(slam_pose); 
}

void SimpleLocalization::onNaviMsgRecvd(const custom_msgs::NaviData::ConstPtr &msg) 
{
    int gnss_status;
    std_msgs::Float32 speed;

    gnss_status = msg->pose_type;
    speed.data = msg->speed2d;
    speed_pub.publish(speed);   //发布速度

    //由惯导发布tf to map
    getPoseonMap(msg);      //经纬度转xy   得到了惯导/cur_pose
    yawAngle = Eigen::AngleAxisd((theta * M_PI) / 180, Eigen::Vector3d::UnitZ());   

    static int print_times=0;   // 打印 fre
    print_times++; 
    if(print_times%20==0)
         cout << "gnss: " << cur_pose.x << " " << cur_pose.y << " " << cur_pose.theta <<endl; 
    if (is_save_trajectory) 
        fout1  << cur_pose.x << " " << cur_pose.y << std::endl;       

    ros::param::get("/Is_slam_start", Is_slam_start);  
    ros::param::get("/slam_origin_get_once", slam_origin_get_once);  
    // transformStamped = tf_buffer_ptr->lookupTransform("map", "map", ros::Time(0));     //test：获取一次会不会保持
    // cout << transformStamped.transform.rotation.w << " "
    //     << transformStamped.transform.rotation.x << " "
    //     << transformStamped.transform.rotation.y << " "
    //     << transformStamped.transform.rotation.z << endl; 
    ros::Time temp_time = ros::Time::now();     //统一时间源
    
    if (Is_slam_start != 0) {        //TASK already start
 
        if ( Is_slam_start==1 && slam_origin_get_once==false )    
        {

            try {   
                transformStamped = tf_buffer_ptr->lookupTransform("map", "base_link", ros::Time(0)); //map是slam map
                // transformStamped.header.frame_id = "gnss_map"; 
                // transformStamped.child_frame_id = "map"; 
                // static_br.sendTransform(transformStamped); 
                TransformToMatrix(transformStamped, Tgo_so); 
                ros::param::set("/slam_origin_get_once", true);     //获取完关系
                ros::param::get("/slam_origin_get_once", slam_origin_get_once); 
                ros::param::set("/Is_slam_start", 2);     //获取完关系   for  TASK   SIGNAL 
                ros::param::get("/Is_slam_start", Is_slam_start);                 
                cout << "slam map origin: " << transformStamped.transform.translation.x << " " << transformStamped.transform.translation.y << endl;

            }
            catch (const std::exception &e)
            {   
                std::cerr << e.what() << std::endl;
                ros::param::set("/slam_origin_get_once", false);  
            } 

        }

        if( slam_origin_get_once==true && print_times%40==0 )     
            std::cout << "slam map origin: " << transformStamped.transform.translation.x            \
                            << " " << transformStamped.transform.translation.y << std::endl;  

        if( slam_origin_get_once==true && Is_slam_start==3 )        // slam localization
        // if (slam_origin_get_once==true && Is_slam_start==3 && (gnss_status!=42 || gnss_status!=52)) 
        {
            cur_pose.x = slam_pose.x;   
            cur_pose.y = slam_pose.y;  
            // cur_pose.theta = slam_pose.theta;        // 注释掉：暂时考虑只用惯导的偏航
            // cur_pose.theta = 0.55*slam_pose.theta + 0.45*cur_pose.theta;      // 互补滤波器，融合slam与gnss的偏航
            // yawAngle = Eigen::AngleAxisd( map_yaw-aftYaw, Eigen::Vector3d::UnitZ() );      //弧度

            if (is_save_trajectory) {
                fout2  << cur_pose.x << " " << cur_pose.y << std::endl;
            }

        }


    }

    /********************************************************************/
    /***************************common***********************************/
    /********************************************************************/
    try     
    { 
        quaternion = yawAngle * pitchAngle * rollAngle;         // 采用的zyx旋转
        br.sendTransform(tf::StampedTransform( 
        tf::Transform(tf::Quaternion(   quaternion.x(),
                                        quaternion.y(),
                                        quaternion.z(),
                                        quaternion.w()),
                        tf::Vector3(cur_pose.x, cur_pose.y, 0)).inverse(),
                        temp_time, "base_link", "map") ); 

        // transformStamped.transform.rotation.w = 1;
        // transformStamped.transform.rotation.x = 0;
        // transformStamped.transform.rotation.y = 0;
        // transformStamped.transform.rotation.z = 0;
        // transformStamped.transform.translation.x = 0;
        // transformStamped.transform.translation.y = 0;
        // transformStamped.transform.translation.z = 0;
        // transformStamped.header.frame_id = "gnss_map";
        // transformStamped.child_frame_id = "map"; 
        // static_br.sendTransform(transformStamped); 

        // br.sendTransform(tf::StampedTransform(
        // tf::Transform(tf::Quaternion( 0, 0, 0, 1),   //单位四元素
        //                     tf::Vector3(0, 0, 0)),
        //                     temp_time, 
        //                     "gnss_map", "map") );                  


    } catch (const std::exception &e) 
    {
        std::cerr << e.what() << std::endl; 
    }

    chatter_pub.publish(cur_pose); //发布

    if (is_display_trajectory) 
    { 
        try{
            listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("map", "base_link", ros::Time(0), transform); 

        }catch (tf::TransformException &ex) { 
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }  
        geometry_msgs::Point p;
        //选择定位数据进行填充
        p.x = transform.getOrigin().x();
        p.y = transform.getOrigin().y();
        p.z = 0;
        path_marker.points.push_back(p); 
		if(path_marker.points.size() > 500)
            path_marker.points.erase(path_marker.points.begin());

        if (Is_slam_start==0)      //show
            path_marker.color = lawngreen; 
        else if(Is_slam_start ==3)
            path_marker.color = white; //slam轨迹颜色
        path_pub.publish(path_marker); 

        //save points
        if (is_save_trajectory) {
            fout3  << p.x << " " << p.y << std::endl;
        }
            
    }   

    ros::spinOnce(); 
}

void SimpleLocalization::getPoseonMap(const custom_msgs::NaviData::ConstPtr &msg) 
{

    VehPose pose_gps(msg->longitude, msg->latitude, msg->heading);  // /navi_msg
    trans_pose_gps2coord(map_ori, pose_gps, pos_map); //坐标原点，经纬度，存放转换后的xy

    cur_pose.x = pos_map.x_lon;
    cur_pose.y = pos_map.y_lat;
    cur_pose.theta = pos_map.yaw_heading;       // 进行了归一化范围后的 yaw_heading

    theta = map_yaw - msg->heading;     // 原始heading   旋转方向反向
}

//转换矩阵获取：ros==>Eigen的接口 
bool SimpleLocalization::TransformToMatrix(const geometry_msgs::TransformStamped& trans, Eigen::Matrix4f& transform_matrix) {  

    double roll, pitch, yaw;
	tf::StampedTransform transform;
	//
	transform.setOrigin( tf::Vector3(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z) );
	transform.setRotation( tf::Quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w) );
    //平移 
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //姿态
    tf::Matrix3x3( transform.getRotation() ).getEulerYPR(yaw, pitch, roll); //ros中的角

    Eigen::AngleAxisf rot_x_btol( roll, Eigen::Vector3f::UnitX() );     //转为Eigen中的旋转向量， 以x轴旋转roll弧度
    Eigen::AngleAxisf rot_y_btol( pitch, Eigen::Vector3f::UnitY() ); 
    Eigen::AngleAxisf rot_z_btol( yaw, Eigen::Vector3f::UnitZ() ); 

    //TF变换转换到变换矩阵 
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();  //右乘

    return true; 
}



