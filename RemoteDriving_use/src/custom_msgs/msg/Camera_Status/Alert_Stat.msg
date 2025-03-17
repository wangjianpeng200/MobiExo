

#-----相机警报----------

uint8 DeviceStatus      ##//设备状态
uint32 AdasAlertStatus  ## //ADAS报警信息标志位 一共用到前五位，第一位:LDW, 第二位：ISL, 第三位 
                             ##   //第四位：PCW一级, 第五位：PCW二级,第六位：FCW一级，第七位：FCW二级，第八位HLW
uint8 FCW_ObjId         ## //FCW目标ID
uint8 PCW_ObjId         ## //PCW目标ID
uint8 LDW_Position      ### //LDW预警位置
uint8 ISL_Value          #//限速牌值
uint8 HLW_Value         ## //限高牌值
uint8 HMW_ObjId        ## //HMW目标ID
