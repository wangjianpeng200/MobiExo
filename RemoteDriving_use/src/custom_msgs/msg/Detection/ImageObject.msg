#------------------------------
#通过图像检测出的目标
#------------------------------
int8        id
int8        type                        #目标类型[待定]
int8        threatLevel
uint8        color_r
uint8        color_g
uint8        color_b
geometry_msgs/Vector3       lwh         #x,y,z分别为目标的长宽高，待定
float32     x_pos                       #x位置，待定
float32     y_pos                       #y位置，待定
float32     z_pos                       #y位置，待定
