#------------------------------
#经过滤波后的目标
#------------------------------


uint32      id                          #目标id
geometry_msgs/Point[8]    bbox_point    #目标边框的8个顶点
int8        type                        #目标类型[-1，unknown][1,car][2,pedestrian]

float32     vx                          #x方向速度，m/s
float32     vy                          #y方向速度，m/s
float32     v                           #总速度，km/h


geometry_msgs/Vector3       lwh         #x,y,z分别为目标的长宽高,待定
float32     x_pos                       #x位置，待定
float32     y_pos                       #y位置，待定

float32     s_pos                       #s位置
float32     d_pos                       #d位置

float32     lidar_x_pos                 #储存雷达坐标，用于目标跟踪
float32     lidar_y_pos
float32     lidar_z_pos