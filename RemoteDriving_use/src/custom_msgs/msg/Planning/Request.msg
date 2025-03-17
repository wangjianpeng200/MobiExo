#---------------------------------------------------------------
# 作者 ：杨东
# 时间 ：2019.08.12
# 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
# 说明 ：
#----------------------------------------------------------------

std_msgs/Header head


uint8 FORWARD_ENABLE=0     #前进使能
uint8 BACK_ENABLE=1        #倒车使能
uint8 STOP_ENABLE=2        #停车使能
uint8 AEB_ENABLE=3         #紧急刹车使能

uint8    reques_type       #使能类型
float32  run_speed         #行车速度
float32  stop_distance     #前方停止距离
float32  aeb_distance      #前方AEB停止距离


