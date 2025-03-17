#include <iostream>
#include <pqxx/pqxx>
#include <cmath>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <string>
#include "map.h"

using namespace pqxx;
using namespace std;
string dbname;
string user;
string password;
string hostaddr;
string port;
string address;
string angle;  
int ID_SUM;
static int get_11_road_id=0;

void ConfigFileRead()
{
    ifstream configFile;
    const char* env_p = std::getenv("SEED_HOME");
    string path = env_p;   
    path += "/setting.conf";
    configFile.open(path.c_str());
    string strLine;
    if(configFile.is_open())
    {
        while (!configFile.eof())
        {
            getline(configFile, strLine);
            size_t pos = strLine.find('=');
            string key = strLine.substr(0, pos);

            if (key == "dbname") {
                dbname=strLine.substr(pos + 1);
            } else if(key == "user"){
                user = strLine.substr(pos + 1);
            } else if(key == "password") {
                password = strLine.substr(pos + 1);
            } else if(key == "hostaddr") {
                hostaddr = strLine.substr(pos + 1);
            } else if(key == "port") {
                port = strLine.substr(pos + 1);
            } else if(key == "angle"){
                angle = strLine.substr(pos + 1);
            } else if(key == "address"){
                address = strLine.substr(pos + 1);
            }else if(key =="mapidsum"){
                ID_SUM = std::stoi(strLine.substr(pos + 1));
            }
        }
    }
    else
    {
        cout << "Cannot open config file!" << endl;
    }
    
}


void GetLaneBEP(const int lane_id, vector<LanePoint> &LanePoints){
    ConfigFileRead();
    string tablename = "reference_linkpoints_" + address;
    LanePoint point;
    try
    {
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
           //cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            //cout << "Can't open database" << endl;
        }
        string sql1 = "select pointorder,orig_fid,point_x,point_y from "+ tablename +" where orig_fid="+to_string(lane_id) + "order by orig_fid, pointorder";
        work N(C);
        result R( N.exec( sql1 ));
        N.commit();
        for (result::const_iterator c = R.begin(); c != R.end(); ++c)
        {
            point.sortt = c[0].as<int>();
            point.road_id = c[1].as<int>();
            point.x = c[0].as<double>();
            point.y = c[1].as<double>();
            LanePoints.push_back(point);
        }

        //cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
	    cerr<<e.what() << std::endl;
    }
}

int GetLaneCnt(){
    ConfigFileRead();
    string tablename = "reference_link_" + address;
    int cnt;
    try
    {
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            //cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            //cout << "Can't open database" << endl;
        }
        string sql = "SELECT max(id) from "+ tablename;
        work N(C);
        result R( N.exec( sql ));
        N.commit();
        for (result::const_iterator c = R.begin(); c != R.end(); ++c)
        {
            cnt = c[0].as<int>();
        }
        //cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
	    cerr<<e.what() << std::endl;
    }
    return cnt;
}


vector<LanePoint> get_referencr_point_zqy(int Lane_number){
    vector<LanePoint> points;

    LanePoint point;
	LanePoint point_after_tran;
    string sql;

    ConfigFileRead();
    string tablename = "reference_linkpoints_"+address;
    double angle_flout = atof(angle.c_str());
    try
    {
        // int Lane_number_next = Lane_number+1;
        // if(Lane_number == ID_SUM){ 
        //     Lane_number_next = 1;
        // }
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            //cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
           // cout << "Can't open database" << endl;
        }
        sql = "SELECT pointorder,orig_fid,point_x,point_y from "+ tablename +" where orig_fid="+to_string(Lane_number) +" order by pointorder";
        work N1(C);
        result R1( N1.exec( sql ));
        N1.commit();
        for (result::const_iterator c = R1.begin(); c != R1.end(); ++c)
        {
            //std::cout<<"参考线 本段id----------"<<Lane_number<<std::endl;
            point.sortt = c[0].as<int>();
            point.road_id = c[1].as<int>();
            point.x = c[2].as<double>();
            point.y = c[3].as<double>();
            double new_x = cos(angle_flout * M_PI / 180) * point.x - sin(angle_flout * M_PI / 180) * point.y;
            double new_y = sin(angle_flout * M_PI / 180) * point.x + cos(angle_flout * M_PI / 180) * point.y;
            point.x = new_x;
            point.y = new_y;
            points.push_back(point);
        }

        // sql = "SELECT pointorder,orig_fid,point_x,point_y from "+ tablename +" where orig_fid="+to_string(Lane_number_next) +" order by pointorder";
        // work N2(C);
        // result R2( N2.exec( sql ));
        // N2.commit();
        // std::cout<<"Lane_number_next = "<<Lane_number_next<<std::endl;
        // for (result::const_iterator c = R2.begin(); c != R2.end(); ++c)
        // {
        //     //std::cout<<"参考线 后一段id----------"<<Lane_number_next<<std::endl;
        //     point.sortt = c[0].as<int>();
        //     point.road_id = c[1].as<int>();
        //     point.x = c[2].as<double>();
        //     point.y = c[3].as<double>();
        //     double new_x = cos(angle_flout * M_PI / 180) * point.x - sin(angle_flout * M_PI / 180) * point.y;
        //     double new_y = sin(angle_flout * M_PI / 180) * point.x + cos(angle_flout * M_PI / 180) * point.y;
        //     point.x = new_x;
        //     point.y = new_y;
        //     points.push_back(point);
        // }
    
        
        //cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
	cerr<<e.what() << std::endl;
    }
    return points;
}


// tablename:  lanelink_leftpoint_zqy or lanelink_rightpoint_zqy
vector<LanePoint> get_lanelink_point_zqy(int Lane_number, string tablename){
    vector<LanePoint> points;
    LanePoint point;
	LanePoint point_after_tran;
    string sql;
    ConfigFileRead();
    double angle_flout = atof(angle.c_str());
    try
    {
        // int Lane_number_next = Lane_number+1;
        // if(Lane_number == ID_SUM){
        //     Lane_number_next = 1;
        // }
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            //cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            //cout << "Can't open database" << endl;
        }
        
        sql = "SELECT pointorder,orig_fid,point_x,point_y from "+ tablename +" where orig_fid ="+to_string(Lane_number)+" order by pointorder";
        work N1(C);
        result R1( N1.exec( sql ));
        N1.commit();
        for (result::const_iterator c = R1.begin(); c != R1.end(); ++c)
        {
            point.sortt = c[0].as<int>();
            point.road_id = c[1].as<int>();
            point.x = c[2].as<double>();
            point.y = c[3].as<double>();
            
            double new_x = cos(angle_flout * M_PI / 180) * point.x - sin(angle_flout * M_PI / 180) * point.y;
            double new_y = sin(angle_flout * M_PI / 180) * point.x + cos(angle_flout * M_PI / 180) * point.y;
            point.x = new_x;
            point.y = new_y;
            points.push_back(point);
        }

    //     sql = "SELECT pointorder,orig_fid,point_x,point_y from "+ tablename +" where orig_fid ="+to_string(Lane_number_next)+" order by pointorder";
    //     work N2(C);
    //     result R2( N2.exec( sql ));
    //    N2.commit();
    //     for (result::const_iterator c = R2.begin(); c != R2.end(); ++c)
    //     {
    //         point.sortt = c[0].as<int>();
    //         point.road_id = c[1].as<int>();
    //         point.x = c[2].as<double>();
    //         point.y = c[3].as<double>();
            
    //         double new_x = cos(angle_flout * M_PI / 180) * point.x - sin(angle_flout * M_PI / 180) * point.y;
    //         double new_y = sin(angle_flout * M_PI / 180) * point.x + cos(angle_flout * M_PI / 180) * point.y;
    //         point.x = new_x;
    //         point.y = new_y;
    //         points.push_back(point);
    //     }
    
        
        //cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
	cerr<<e.what() << std::endl;
    }
    return points;
}


Lane_Message select_Lane_by_id(int id)
{
    string sql;
    string tablename;
    Lane_Message lane;
    ConfigFileRead();
    try
    {
        tablename = "reference_link_" + address;
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            //cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            //cout << "Can't open database" << endl;
        }
        sql = "SELECT id,turn_flag,velocity,scenes,start_x,start_y,end_x,end_y,lane_num from "+tablename+" where id=" + to_string(id);
        nontransaction N(C);
        result R( N.exec( sql ));
        for (result::const_iterator c = R.begin(); c != R.end(); ++c)
        {
            lane.current_lane_id = c[0].as<int>();
            
            if (ID_SUM==1){     //解决
                lane.next_lane_id = 1;
            }else if(lane.current_lane_id >= ID_SUM)  // 如果只有1段就会出问题
                lane.next_lane_id = (lane.current_lane_id+1)%ID_SUM;
            else 
                lane.next_lane_id = lane.current_lane_id+1;
            
            lane.LanePoints_right_current_line = get_lanelink_point_zqy(lane.current_lane_id, "lanelink_rightpoints_"+address);
            lane.LanePoints_left_current_line = get_lanelink_point_zqy(lane.current_lane_id, "lanelink_leftpoints_"+address);
            lane.ReferencePoints_current_line = get_referencr_point_zqy(lane.current_lane_id);
            lane.turn_flag = c[1].as<int>();
            lane.current_lane_num = c[8].as<int>();// should be modify
            lane.v = c[2].as<int>();
            lane.scenes_id = c[3].as<int>();
            lane.current_lane_start_point.x = c[4].as<double>();
            lane.current_lane_start_point.y = c[5].as<double>();
            lane.current_lane_end_point.x = c[6].as<double>();
            lane.current_lane_end_point.y = c[7].as<double>();

        }
        //cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
        cerr << e.what() << std::endl;
    }
    return lane;
}

int Get_Lane_id_by_Coor(double point_x, double point_y) {
    string sql;

    int current_lane_id;
    LanePoint lane_point;
    double min_length;
    ConfigFileRead();
    try
    {
        connection C("dbname="+dbname+" user="+user+" password="+password+" hostaddr="+hostaddr+" port="+port);
        if (C.is_open())
        {
            //cout << "Opened database successfully: " << C.dbname() << endl;
        }
        else
        {
            ///cout << "Can't open database" << endl;
        }
        sql = "SELECT orig_fid,point_x,point_y from reference_linkpoints_"+address;
        nontransaction N(C);
        result R( N.exec( sql ));

        min_length = 10000.0;


        for (result::const_iterator c = R.begin(); c != R.end(); ++c)
        {
            lane_point.x = c[1].as<double>();
            lane_point.y = c[2].as<double>();
            double length = sqrt((lane_point.x - point_x) * (lane_point.x - point_x) + (lane_point.y - point_y) * (lane_point.y - point_y));
            if (length < min_length){
                current_lane_id = c[0].as<int>();
                min_length = length;
            }
        }
//
        //cout << "Operation done successfully" << endl;
        C.disconnect ();
    }
    catch (const std::exception &e)
    {
        cerr << e.what() << std::endl;
    }
    return current_lane_id;
}





