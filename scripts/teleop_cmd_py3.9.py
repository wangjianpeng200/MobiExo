from pathlib import Path
import argparse
import yaml
import numpy as np
from typing import Dict, Any
import math
import time
import socket

import ace_teleop
from ace_teleop.control.teleop import ACETeleop
from Robotic_Arm.rm_robot_interface import *

start=1

class Gen_72:
    def __init__(self, ip):
        #------------------------------------变量初始化
        self.ip = ip
        self.left_joint=[0.0]*7
        self.left_gipper_two=[0.0]*2
        self.right_joint=[0.0]*7
        self.right_gripper_two=[0.0]*2
        # self.left_basejoint=[0.27377135, -0.74260074, -0.3113867 , -2.1312518,  -0.21240002,  1.006088 ,2.4039316] #左臂初始位置
        # self.right_basejoint=[-0.27377135, -0.74260074, 0.3113867 , -2.1312518,  0.21240002,  1.006088 ,2.0039316]#右臂初始位置
        # self.left_basejoint=[0.35732433,-0.6933105 ,-0.44294065,-1.9173014 ,-0.17056961,0.869710862,0.2781827] 
        # self.right_basejoint=[-1.017996 ,-0.9553843 ,0.55155945 ,-1.8001652 , 0.809958  ,0.43882862 ,2.1733184]
        self.left_basejoint=[1.3986722 , -1.4858838 , -1.6482577 , -1.6311567 ,  0.0713942  , -0.11811505 , 0.7027218] 
        self.right_basejoint=[1.7444494  , 1.4980569 , -1.50291 ,-1.6323614  ,-0.06108665 ,-0.11514272 , 0.8577711]
        self.right_gripper=[0.0]
        self.left_gripper=[0.0]
        self.right_gipflag=0
        self.left_gipflag=0

        #-------------------------------------API初始化以及socket连接
        # 创建机械臂对象,三线程
        self.right_arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)# 创建机械臂对象,三线程
        self.left_arm = RoboticArm()
        right_handle = self.right_arm.rm_create_robot_arm("192.168.1.18", 8080)# 创建机械臂连接，打印连接id
        left_handl= self.left_arm.rm_create_robot_arm("192.168.1.19", 8080)
        # print(right_handle.id)
        # print(left_handl.id)
        self.right_arm.rm_set_arm_run_mode(1)#右臂设置为仿真模式
        self.left_arm.rm_set_arm_run_mode(1)#右臂设置为仿真模式

        #-----------------------------------机械臂,夹爪,控制模式，modbus初始化
        self.right_arm.rm_set_tool_voltage(3) #末端工具电压设置为24v
        self.left_arm.rm_set_tool_voltage(3) 
        for i in range(7):
            self.right_basejoint[i]=math.degrees(self.right_basejoint[i])
        for i in range(7):
            self.left_basejoint[i]=math.degrees(self.left_basejoint[i])

        # base_rightflag=self.right_arm.rm_movej(self.right_basejoint, 30, 0, 0, 0)    # 右臂运动到初始位置
        # self.left_arm.rm_movej(self.left_basejoint, 30, 0, 0, 0)                     #左臂运动到初始位置

        self.right_arm.rm_set_modbus_mode(1,115200,2)   #modbus初始化
        self.left_arm.rm_set_modbus_mode(1,1115200,2)

        self.right_write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
        self.left_write_params = rm_peripheral_read_write_params_t(1, 40000, 1)

        self.right_arm.rm_write_single_register(self.right_write_params,100) #设置modbus写入参数，打开夹爪
        self.left_arm.rm_write_single_register(self.left_write_params,100)
        time.sleep(5)

    #---------------------------------------获取右机械臂和夹爪状态  
    def right_get(self,cmd: np.ndarray):
        right_joint_rad=cmd[9:16]
        for i in range(7):
            self.right_joint[i]=math.degrees(right_joint_rad[i])
        # print("right_joint:",self.right_joint)
        self.right_gripper_two=cmd[16:18]
        # print("right_gripper:",self.right_gripper)
        
    #---------------------------------------获取左机械臂和夹爪状态
    def left_get(self,cmd: np.ndarray):
        left_joint_rad=cmd[0:7]
        for i in range(7):
            self.left_joint[i]=math.degrees(left_joint_rad[i])
        self.left_gripper_two=cmd[7:9]
        # print("left_gripper:",self.left_gripper)

    #---------------------------------------夹爪数据处理
    def process_gripper(self):
        self.right_gripper=(self.right_gripper_two[0]/0.04)*100
        self.left_gripper=(self.left_gripper_two[0]/0.04)*100

    #---------------------------------------夹爪阈值判断
    def threshold_gripprt(self,right_gipper:np.ndarray,left_gipper:np.ndarray):
        if (right_gipper <21 ) and (self.right_gipflag == 1):
            self.right_arm.rm_write_single_register(self.right_write_params,10)
            self.right_gipflag=0
        #状态为闭合，且需要张开夹爪
        if (right_gipper>79) and (self.right_gipflag== 0):
            self.right_arm.rm_write_single_register(self.right_write_params,100)
            self.right_gipflag=1

        if (left_gipper <21 ) and (self.left_gipflag == 1):
            self.left_arm.rm_write_single_register(self.left_write_params,10)
            self.left_gipflag=0
        #状态为闭合，且需要张开夹爪
        if (left_gipper>79) and (self.left_gipflag== 0):
            self.left_arm.rm_write_single_register(self.left_write_params,100)
            self.left_gipflag=1

    def run_init(self,cmd: np.ndarray):
        self.right_get(cmd)
        self.left_get(cmd)
        self.right_arm.rm_movej(self.right_joint, 30, 0, 0, 0)    # 右臂运动到初始位置
        self.left_arm.rm_movej(self.left_joint, 30, 0, 0, 0)                     #左臂运动到初始位置

        
    #---------------------------------------运行
    def run(self,cmd: np.ndarray):
        self.right_get(cmd)
        self.left_get(cmd)
        self.process_gripper()
        self.right_arm.rm_movej_canfd(self.right_joint,True, 0, 1, 50)
        self.left_arm.rm_movej_canfd(self.left_joint,True, 0, 1, 50)
        self.threshold_gripprt(self.right_gripper,self.left_gripper)
    
    def __del__(self):
        self.left_arm.rm_delete_robot_arm()
        self.right_arm.rm_delete_robot_arm()


def start_receiver():
# 创建一个 UDP 套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    local_address = ("127.0.0.1", 12345)
    udp_socket.bind(local_address)
    print("Waiting for start message...")
    while True:
        data, addr = udp_socket.recvfrom(1024)
        message = data.decode('utf-8')
        # 检查是否为 "start" 消息
        if message == "start":
            start = 1
            print(f"Received start message from {addr}, start is set to {start}")
            break
    # 关闭套接字
        udp_socket.close()

def load_config(config_file_name: str) -> Dict[str, Any]:
    robot_config_path = (
        f"{ace_teleop.__path__[0]}/configs/robot" / Path(config_file_name)
    )
    with Path(robot_config_path).open("r") as f:
        cfg = yaml.safe_load(f)["robot_cfg"]

    return cfg

def main() -> None:
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        "-c",
        choices=["h1_inspire", "xarm_ability", "gr1", "franka"],
        default="h1_inspire",
    )
    parser.add_argument("--ip", default="localhost")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    config_file_name = f"{args.config}.yml"
    cfg = load_config(config_file_name)

    teleoperator = ACETeleop(cfg, args.ip, debug=args.debug)
    start_receiver()
    if start==1:
        gen72=Gen_72("192.168.1.18") 
        time.sleep(1)
        for _ in range(10):
            cmd = teleoperator.step()
            print("cmd:", cmd)
        gen72.run_init(cmd)
        time.sleep(4)
        try:
            while True:
                start_time = time.time()  # 记录开始时间

                if args.debug:
                    cmd, latest = teleoperator.step()
                else:
                    cmd = teleoperator.step()
                    gen72.run(cmd)
                    # print("Non-debug mode cmd:", cmd)

                end_time = time.time()  # 记录结束时间
                frame_time = end_time - start_time
                frame_rate = 1.0 / frame_time if frame_time > 0 else float('inf')
                print(f"Frame Time: {frame_time:.6f} seconds, Frame Rate: {frame_rate:.2f} FPS")
        except KeyboardInterrupt:
            exit(0)

if __name__ == "__main__":
    main()

