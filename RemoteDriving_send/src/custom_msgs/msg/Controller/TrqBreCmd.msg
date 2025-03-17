#---------------------------------------------------------------
# 作者 ：杨东
# 时间 ：2019.08.12
# 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
# 说明 ：
#----------------------------------------------------------------

std_msgs/Header head

#float32 DeAccReq      
#float32 AccReq
#float32 SptReq

uint8   trq_enable                    #发动机驱动扭矩是否使能
uint32  trq_value_2                   #->>扭矩:所需扭矩值     #2号车使用
float32 trq_value_3                   #->>扭矩:所需扭矩百分比  #3号车使用
uint8   bre_enable                    #刹车是否使能
float32 bre_value                     #刹车值
uint8   ACC_DecToStop                 #ACC请求减速到停止 0x0: no demand; 0x1: demand 车子按自己的减速度平滑停车 不受发送的其他指令控制
uint8   ACC_Driveoff                  #释放汽缸压力，必须与ACC_DecToStop配合使
