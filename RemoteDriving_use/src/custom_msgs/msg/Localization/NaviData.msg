#---------------------------------------------------------------------------------------------
#导航信息
#来源于GNSS，INS设备

# 50 NARROW_INT Integer narrow-lane ambiguity solution
# 56 INS_RTKFIXED INS RTK fixed ambiguities solution
# 51 RTK_DIRECT_INS RTK status where the RTK filter is directly initialized from the INS filter
# 55 INS_RTKFLOAT INS RTK floating point ambiguities solution
# 52 INS_SBAS INS calculated position corrected for the antenna
# 54 INS_PSRDIFF INS pseudorange differential solution
# 53 INS_PSRSP INS pseudorange single point solution – no DGPS corrections
#-----------------------------------------------------------------------------------------------


std_msgs/Header head

float32 pitch          #俯仰角
float32 roll           #翻滚角
float32 heading        #航向角

float64 longitude      #经度
float64 latitude       #纬度
float64 altitude       #海拔

float32 speed2d        #车辆速度
# ----------------->
float32 Ve

float32 Vn

float32 Vu

# <-----------------

int32   pose_type      #定位状态
float64 gpsTime        #

int32 INS_Status       #解算状态
float32 Lat_vari       #纬度标准差
float32 Lon_vari       #经度标准差