//********************************************************************************************
// 作   者-->:  杨 东
// 创建时间-->:  2019.11.12
// 修改时间-->:  2019.11.12
// 版   权-->:  重庆邮电大学\自动化学院\汽车电子工程中心\智能汽车技术研究所
//--------------------------------------------------------------------------------------------
// 文档说明 ：软件DTU
//         参考网址:
//         https://blog.csdn.net/working24hours/article/details/88323465
//         https://blog.csdn.net/hanford/article/details/53025771
//**********************************************************************************************


#ifndef DTU_API_H
#define DTU_API_H

#include "socket_api.h"
#include <stdint.h>

struct NtripConf{
    char *ip;           //服务器IP  千寻 60.205.8.49
    int  port;          //端口号    8002 对应WGS84
    char *usr_id;       //用户名   若是获取 source_table /  usr_id = "";
    char *pass_wd;      //用户密码 若是获取 source_table /  pass_wd = "";
    char *mountpoint;   //挂载点   若是获取 source_table /  mountpoint = ""; RTCM32_GGB
};

/**
 * @brief set_ntrip_conf
 * @param conf
 * @param msg_request
 * @param len
 */
void set_ntrip_conf(struct NtripConf &conf,char *msg_request,u_int32_t len);

/**
 * @brief ntrip_caster_source_table
 * @param conf
 */
void ntrip_caster_source_table(struct NtripConf &conf);

#endif // DTU_API_H
