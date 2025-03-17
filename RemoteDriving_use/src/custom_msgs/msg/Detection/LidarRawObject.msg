#------------------------------
#通过激光雷达检测出的目标
#------------------------------


geometry_msgs/Point[8]    bbox_point  #目标边框的8个顶点


geometry_msgs/Vector3     lwh         #x,y,z分别为目标的长宽高，待定
float32                   x_pos       #x位置，待定
float32                   y_pos       #y位置，待定
float32                   z_pos       #

float32                   lidar_x_pos       #储存雷达坐标，用于目标跟踪
float32                   lidar_y_pos       
float32                   lidar_z_pos