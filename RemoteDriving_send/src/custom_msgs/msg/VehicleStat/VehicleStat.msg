#---------------------------------------------------------------
# 作者 ：杨东
# 时间 ：2019.08.12
# 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
# 说明 ：
#----------------------------------------------------------------
float32   VehicleSpeed               #车速，单位km/h
int8 GearShiftPositon                #换挡器位置
uint8 ParkSts                        #驻车状态
#ErrorCode 各位为1代表含义如下：
#    0：不能收到can报文
#    1：VcuError不能进入自动驾驶状态
#    2：横向eps故障
#    3：纵向ebs故障
#    4：
#    5：
#    6：
#    7：
uint8 ErrorCode                      #故障码
uint16 RemainingMile                 #剩余里程计
float32 TolMileage                   #累计里程计



float32 veh_speed                       #速度，单位km/h
float32 SAS_SteeringAngle               #方向盘转角速度
float32 EMS_EngineSpeed                 #发动机转速
float32 EMS_EngineThrottlePosition      #发动机节气门位置
float32 EMS_AccPedal                    #加速踏板位置
uint8   EMS_BrakePedalStatus            #制动踏板状态
uint8   TCU_GearShiftPositon            #换挡器位置
uint8   VehicleModle                    #车辆状态：自动驾驶or手动驾驶