#ifndef MAP_H
#define MAP_H
#include <vector>
#include <string>
#define ID_SUM 1
using namespace std;


typedef struct LanePoint{
    double x;
    double y;
    int sortt;
    int road_id;
} LanePoint;

typedef struct Lane
{
    int current_lane_id;
    int next_lane_id;
    vector<LanePoint> LanePoints_right_current_line;
    vector<LanePoint> LanePoints_left_current_line;
    vector<LanePoint> ReferencePoints_current_line;
    vector<LanePoint> LanePoints_right_next_line;
    vector<LanePoint> LanePoints_left_next_line;
    vector<LanePoint> ReferencePoints_next_line;
    int turn_flag;
    int current_lane_num;
    int v;
    int scenes_id;
    LanePoint current_lane_start_point;
    LanePoint current_lane_end_point;
} Lane_Message;




void ConfigFileRead();
vector<LanePoint> get_referencr_point_zqy(int Lane_number);
vector<LanePoint> get_lanelink_point_zqy(int Lane_number, string tablename);
Lane select_Lane_by_id(int id);
int Get_Lane_id_by_Coor(double point_x, double point_y);
int GetLaneCnt();
void GetLaneBEP(const int laneid, vector<LanePoint> &LanePoints);

#endif // MAP
