#---------------------------------------------------------------
# 作者 ：杨东
# 时间 ：2019.08.12
# 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
# 说明 ：
#----------------------------------------------------------------

std_msgs/Header head

uint8 APA_TransPRNDShiftEnable         #->>档位：请求换挡可控 0x1 disable able 0x1 Enable   ACM_APA Request Enable = 0x1:Control enabled
uint8 APA_TransPRNDShiftReqValid       #->>档位：换挡使能   0x0 = Invalid 0x1=Valid
uint8 APA_TransPRNDShiftRequest        #->>档位：换挡档位值 0x0=NoRequest;or 0x1=P ;or 0x2= R ; 0x3=N 0x4=D 0x5 :invalid


