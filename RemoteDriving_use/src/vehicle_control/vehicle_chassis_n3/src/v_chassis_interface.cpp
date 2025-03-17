#include <v_chassis_interface.h>
int   i=0;
///-->>静态全局变量------------------------------------------------------------------------------------------------
  static bool self_drive_mode = true;                         //自动驾驶模式
  static struct MSG_ID301 msg_id301 = { 0,0,0,0,0,0,0,0,0,0 }; //转向、制动消息
  static struct MSG_ID302 msg_id302 = { 0,0,0,0,0,0,0,0 };     //纵向消息
  static struct MSG_ID303 msg_id303 = { 0,0,0,0 };             //灯光控制
  static u_int8_t turn_light=0;
  int can_fd = 0;
  float kp=0,ki=0,kd=0;
//-->>状态反馈报文-------------------------------------------------------------------------------------------------

  static struct MSG_ID3f1 msg_id3f1 = { 0,0,0 };   //车辆转向与制动状态反馈
  static struct MSG_ID3f2 msg_id3f2 = { 0,0,0 };   //车辆纵向消息
  static struct MSG_ID3f3 msg_id3f3 = { 0,0,0,0 }; //车辆灯光、喇叭消息


///----------------------------------------------------------------------------------------------------------------
///该部分主要实现了外部通过消息或者服务实现对控制命令的赋值，实现对车辆的控制，主要包含以下内容
///-->>转向、油门、刹车和档位的控制、 AEB大电机的控制
///-->>BCM的相关控制：门锁控制、雨刮控制、喇叭、近光灯命令、远光灯命令、转向灯命令、示廓灯命令、前雾灯命令、后雾灯命令
///-->>外部获取车辆状态的实现
/// ----------------------------------------------------------------------------------------------------------------


void vehicle_ready(){
    msg_id301.ACU_Brake_Conway    = 1; //制动减速度控制
    msg_id302.ACU_M_Conway        = 0; 
}

void vehicle_clear(){
  if (msg_id3f1.VCU_Switch_sta == 0){
    msg_id301 = { 0,0,0,0,0,0,0,0,0,0 }; //转向、制动消息
    msg_id302 = { 0,0,0,0,0,0,0,0 };     //纵向消息
    msg_id303 = { 0,0,0,0 };             //灯光控制
  }
}
  /**
   * @brief get_veh_status
   * @param stutus
   */
  void get_veh_status(struct VehicleStat &stutus){
      stutus.VehicleSpeed               = msg_id3f2.VCU_Speed_sta;
      stutus.SAS_SteeringAngle          = msg_id3f1.VCU_SteerAngel_sta;
      stutus.VcuBrkPedlSts              = msg_id3f1.VCU_BrakeValue_sta;
      stutus.TCU_GearShiftPositon       = msg_id3f2.VCU_Gear_sta;
      return ;
  }

 /**
   * @brief get_TurnLight_status
   * @param stutus
   */
  void get_TurnLight_status(struct TurnLight_status &status){
      status.Veh_LowHighLight = msg_id3f3.VCU_LowHighLight_sta;
      status.Veh_TurnLightLe  = msg_id3f3.VCU_TurnLight_sta;
      status.Veh_BrakeLight   = msg_id3f3.VCU_BrakeLight_sta;
      status.Veh_VU_Horn      = msg_id3f3.VCU_VB_Horn_sta;
      return ;
  }






//-----------------------------------------------------------------------------------------------------------------
  //--行驶控制外部接口
  /**
   * @brief set_steering_cmd
   * @param angle
   */
float last_error = 0;

  float calculatePID(const float KP,const float KI,const float KD,float angel1){
    float error = fabs(angel1 - msg_id3f1.VCU_SteerAngel_sta);
    ROS_WARN_THROTTLE(0.5,"piderror 22222: %f",error);
    float proportion = KP * (error-last_error);
    float integration = KI *(integration + error);
    float differential = KD * (error - 2 * last_error + integration);
    float increment = proportion + integration + differential;
    ROS_WARN_THROTTLE(0.5,"steer_enable 22222: %f", increment);
    if (increment > 580)  increment = 580;
    else if (increment < 0) increment = 0;
    last_error = error;
    return increment;
  }


  void set_steering_cmd(const float angle){
    ROS_INFO_THROTTLE(2,"lat APA_SteeringAngleRequest= %f",angle);
    // 对角度进行范围检查以后，赋值处理 【-580，,580】
    msg_id301.ACU_SteerEn = 1;
    msg_id301.ACU_Steer_angel = (fabs(angle) <= 580.0f ? angle : (fabs(angle)/angle) *580.0f);
    msg_id301.ACU_Steer_sp = 170;

    // msg_id301.ACU_Steer_sp = (int)(calculatePID(kp,ki,kd,angle) * 0.9310);
    return ;
  }


  /**
   * @brief set_break_trq_cmd
   * @param flag_b
   * @param break_v
   * @param flag_t
   * @param torque
   */
  void set_trq_bre_cmd(uint8_t trq_enable,const float trq_value,const uint8_t bre_enable,const float bre_value,const uint8_t DecToStop,const uint8_t Driveoff){
      
      //-->>判断刹车油门是否同时作用
      static int flag=1;

      if(trq_enable == 1 && bre_enable == 1){
        msg_id302.ACU_M_Drive_En = 0;
        msg_id301.ACU_BrakeEn  = 0;
      }
      else {

        if(msg_id3f2.VCU_Speed_sta <= 1 && flag == 1){ //制动后泄压起步
          flag++;
          msg_id302.ACU_M_Drive_En      = trq_enable;
          msg_id302.ACU_M_Drive_value   = 100;
          msg_id301.ACU_Brake_decA      = 0;
          msg_id301.ACU_BrakeEn         = 0;
        }else{
          //-->>油门开度控制
          msg_id301.ACU_BrakeEn          = 0;
          msg_id301.ACU_Brake_decA       = 0;
          msg_id302.ACU_M_Drive_En      = trq_enable;
          msg_id302.ACU_M_Drive_value   = (trq_value >= 0) && (trq_value <= 100) ? trq_value : 0 ; 
          // ROS_WARN_THROTTLE(0.5,"trq_enable : %d", msg_id302.ACU_M_Drive_En);
          ROS_WARN_THROTTLE(0.5,"set_enable : %d", msg_id302.ACU_M_Drive_value);


        }
        if(bre_enable==true){
          //-->>刹车减速度控制
          msg_id301.ACU_BrakeEn          = bre_enable;
          msg_id302.ACU_M_Drive_En       = 0;
          msg_id302.ACU_M_Drive_value    = 0;
          msg_id301.ACU_Brake_decA       = (trq_value >= -9.9) && (trq_value <= 0) ? trq_value : -9.9f;
          ROS_INFO_THROTTLE(0.1,"bre_value : %f", msg_id301.ACU_Brake_decA);
        }
          ROS_WARN_THROTTLE(0.5,"set_enable111111111 : %d", msg_id302.ACU_M_Drive_value);
     
      }

      return ;
  }

  /**
   * @brief set_aeb_cmd
   * @param stop
   * @param aeb_enable
   * @param aeb_bre_value
   */
  void set_aeb_cmd(const uint8_t aeb_enable,const float aeb_bre_value){
      msg_id302.ACU_M_Drive_value           = 0;
      msg_id302.ACU_M_Drive_En              = 0;
      msg_id303.ACU_BrakeLightEn            = 1; //制动灯控制 
      msg_id301.ACU_BrakeEn                 = aeb_enable;
      msg_id301.ACU_Brake_decA  = (aeb_bre_value >= -9.9f) && (aeb_bre_value <= 0.0f) ? aeb_bre_value : -9.9f;
      return ;
  }

  //-------------------->>BCM控制外部接口<<--------------------------
  /**
   * @brief set_Gear_cmd
   * @param Enable              换挡机构 1使能  0x1 Enable   ACM_APA Request Enable = 0x1:Control enabled
   * @param Request             档位控制相关：档位设置“0:P档 / 1:R档 / 2:N档 / 3:D档”
   */

  void Relase_EPB(){
    msg_id301.ACU_BrakeEn      = 0;
    msg_id302.ACU_M_Drive_En   = 1;   //0x1 :enable
    msg_id303.ACU_BrakeLightEn = 0; //制动灯控制
    // ROS_INFO_THROTTLE(1,"relase shake");
  }

  void Request_EPB(){
    msg_id301.ACU_BrakeEn      = 1;
    msg_id302.ACU_M_Drive_En   = 0;   //0x1 :enable
    msg_id303.ACU_BrakeLightEn = 1; //制动灯控制

  }

  void set_Gear_cmd(const uint8_t Enable,uint8_t Request){
      msg_id302.ACU_M_Gear_En    = Enable;   // 换档控制使能 0x1:Enable；
      msg_id302.ACU_M_Gears      = Request;  //目标档位请求 N :0x2  D: 0x3  R :0x1  P:0x0；
      if(Request == msg_id3f2.VCU_Gear_sta){
        msg_id302.ACU_M_Gear_En    = 0;
      }
    return ;
  }

 

  /**
 * @brief set_TurnLight_cmd
 * @param TurnLight
 */
void set_TurnLight_cmd(const uint8_t TurnLight){
    turn_light = TurnLight;
    ROS_INFO_THROTTLE(2,"set_TurnLight_cmd= %d",turn_light);
    switch(turn_light) {
        case 0:
            msg_id303.ACU_TurnLightEn = 0;
            break;
        case 1:
            msg_id303.ACU_TurnLightEn = 1;
            break;
        case 2:
            msg_id303.ACU_TurnLightEn = 2;
            break;
        case 3:
           msg_id303.ACU_TurnLightEn = 3;
            break;
    }
    return;
    
    return;
}


//--------------------------------------------------------------------------------------------------------------
  /**
   * @brief status_monitor_thread_recall
   * @param arg
   * @return
   */
  static void *status_monitor_thread_recall(void *arg);

  /**
   * @brief manual_auto_driving_thread_recall
   * @param arg
   * @return
   */
  static void *manual_auto_driving_thread_recall(void *arg);

  /**
   * @brief msg_send_recall
   * @param signo
   */
  static void  msg_send_recall(int signo);

//--------------------------------------------------------------------------------------------------------------
//-->>以下方法为软件框架，不需要优化
/**
 * @brief start_status_monitor_thread 开启车辆状态监控线程
 * @return
 */
pthread_t start_status_monitor_thread(){
    pthread_t id =0;
    //开启获取车辆状态线程
    if(pthread_create(&id,nullptr,status_monitor_thread_recall,nullptr) != 0 ){
        std::cerr << "create getCarInforThread thread fail!" <<std::endl;
        exit(-1);
    }
    return id;
}



/**
 * @brief start_send_timer开启控制报文发送定时器
 * @param t_ms
 */
void start_send_timer(int t_ms){
    signal(SIGALRM,msg_send_recall);//SIGALRM是信号类型，收到就执行后面的函数
    struct itimerval it ,oldit;
    it.it_value.tv_sec = 0;//第一次执行时间初始时间
    it.it_value.tv_usec = t_ms*1000; //20ms
    it.it_interval.tv_sec = 0;//间隔时间
    it.it_interval.tv_usec = t_ms*1000;//20ms

    if(-1 == setitimer(ITIMER_REAL,&it,&oldit)) //
    {
        perror("msg_send_timer_init failed");
        exit(-1);
    }
    return ;
}





/**
* @brief msg_send_recall
* @param signo
*/
static void msg_send_recall(int signo){
    if(signo != SIGALRM) return;
    vehicle_clear();
    vehicle_ready();
    static struct can_frame frame;                    //发送帧结构
    ROS_INFO_THROTTLE(0.5, "SET_Speed22222222 : %d",msg_id302.ACU_M_Drive_value);

//--------------------------------------------------------------------------------
    //-->>发送转向控制消息
    // msg_id301.ACU_SteerEn = 1;
    // // msg_id301.ACU_Steer_sp = 150;
    // msg_id301.ACU_Steer_angel = 0;
    // // msg_id301.ACU_BrakeEn = 0;
    // msg_id302.ACU_M_Drive_En = 1;
    msg_id302.ACU_M_Gear_En = 1;
    msg_id302.ACU_M_Gears = 3;
    memset(&frame,0,sizeof (frame));                                   //清空缓存区
    frame_encapsulation_ID301(msg_id301,frame);                        //填充帧消息
    //-->>条件编译
    // #ifdef NO_LOCAL_DEBUG
    write_socketcan_frame(can_fd,frame);                               //发送帧消息
    // #endif

    //-->>发送纵向控制消息
    memset(&frame,0,sizeof (frame));                                   //清空缓存区
    frame_encapsulation_ID302(msg_id302,frame);                        //填充帧消息
    //-->>条件编译
    #ifdef NO_LOCAL_DEBUG
    write_socketcan_frame(can_fd,frame);                               //发送帧消息
    #endif

    //-->>发送灯光消息
    memset(&frame,0,sizeof (frame));                                   //清空缓存区
    frame_encapsulation_ID303(msg_id303,frame);                        //填充帧消息
    //-->>条件编译
    #ifdef NO_LOCAL_DEBUG
    write_socketcan_frame(can_fd,frame);                               //发送帧消息
    #endif
    return;
}



/**
 * @brief status_monitor_thread_recall
 * @param arg
 * @return
 */

void * status_monitor_thread_recall(void *arg){
  //方向控制出错错误码
  std::string err[5] = {"OK","take_over","speed_high","angle > 450","angle > 19.9"};
  struct can_frame frame;

  while(true){
      memset(&frame,0,sizeof (struct can_frame));              //清空接收缓存区
      //-->>条件编译
      #ifdef NO_LOCAL_DEBUG
      read(can_fd,&frame,sizeof(frame));
      #endif
      switch (frame.can_id){
      case ID3f1: {
          frame_parsing_ID3f1(frame.data,msg_id3f1);
          ROS_WARN_THROTTLE(1, "Switch_flag : %d",msg_id3f1.VCU_Switch_sta );
          ROS_INFO_THROTTLE(1, "Steer_value11111111 : %f",msg_id3f1.VCU_SteerAngel_sta );
          break;
      }
      case ID3f2: {
          frame_parsing_ID3f2(frame.data, msg_id3f2);
          ROS_INFO_THROTTLE(1, "Veh_Speed22222222 : %f",msg_id3f2.VCU_Speed_sta );
          ROS_INFO_THROTTLE(1, "Veh_trq_value3333333 : %d",msg_id3f2.VCU_DriveValue_sta );
          break;
      }
      case ID3f3: {
          frame_parsing_ID3f3(frame.data, msg_id3f3);  
          break;
      }
          default: break;
      }
  }
  return arg;
}

///-------------------------------------------------------------------------------------------------
///------------------------------实现人机交互的外部接口-------------------------------------------------
///-------------------------------------------------------------------------------------------------

/**
 * @brief udp_client_init
 * @param server_port
 * @return
 */
int udp_client_init(const int server_port){
    /*set server IP and PORT*/
    struct sockaddr_in serveraddr;
    bzero(&serveraddr,sizeof(serveraddr));  //将字符串serveraddr的所有字节清为0

    serveraddr.sin_family = AF_INET;     //sin_family网络协议
    serveraddr.sin_port = htons(server_port);
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);

    //创建套接字
    int client_sock_fd = socket(AF_INET,SOCK_DGRAM,0);   //成功返回0，失败返回-1
    if(client_sock_fd == -1){
        perror("create client_sock_fd failed!");
        exit(-1);
    }

    //如果绑定错误
    if(bind(client_sock_fd,(struct sockaddr *)&serveraddr,sizeof(serveraddr)) == -1){
        perror("udp client socket)fd bind failed1");
        exit(-1);
      }

    return client_sock_fd;
}


/**
 * @brief manual_auto_driving_thread_recall
 * @param arg
 * @return
 */
static void *manual_auto_driving_thread_recall(void *arg){
  int upd_client_fd = udp_client_init(10001);

  //-->>设置目标主机的地址信息
  struct sockaddr_in server_addr;
  bzero(&server_addr,sizeof(server_addr));
  socklen_t server_addr_len = sizeof(server_addr);
  char *server_ip = (char *)"192.168.200.4";  //
  unsigned short server_port = 10001;
  server_addr.sin_family = AF_INET;
  inet_pton(AF_INET,server_ip,&server_addr.sin_addr);      //将点分十进制转换为整数
  server_addr.sin_port = htons(server_port);
  bzero(&(server_addr.sin_zero),8);

  //-->>开辟接收缓存区，并初始化
  char recvbuf[1] = {0};
  bzero(recvbuf,sizeof(recvbuf));
  int recvbuf_len = sizeof(recvbuf);
  while (true) {

  //-->>轮循接收报文
  bzero(recvbuf,sizeof(recvbuf));
  recvfrom(upd_client_fd,recvbuf,recvbuf_len,0,(struct sockaddr*)&server_addr,&server_addr_len);

  //-->>控制模式置位
  self_drive_mode = recvbuf[0] == 48 ? false : true;   //此处存在一个BUG 安卓传回来的状态编码与C++不一致
  cout << "0000000000000000000000000000000000" << endl;
   ROS_INFO_THROTTLE(0.2,"udp mode= %d" ,self_drive_mode);


  }
  return arg;
}
