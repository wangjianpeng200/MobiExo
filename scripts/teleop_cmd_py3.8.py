from pathlib import Path
import argparse
import yaml
import numpy as np
from typing import Dict, Any

import ace_teleop
from ace_teleop.control.teleop import ACETeleop
from Robotic_Arm.rm_robot_interface import *
import ctypes

FLAG_FOLLOW=0 

class Gen_72:
    def __init__(self, ip):

        #------------------------------------变量初始化
        self.ip = ip
        self.left_joint=[0.0]*7
        self.left_gipper_two=[0.0]*2
        self.right_joint=[0.0]*7
        self.right_gripper_two=[0.0]*2
        self.left_basejoint=[0.0]*7 #左臂初始位置
        self.right_basejoint=[0.0]*7#右臂初始位置
        self.right_gripper=[0.0]
        self.left_gipper=[0.0]
        self.right_gipflag=0
        self.left_gipflag=0



        #-------------------------------------API初始化以及socket连接
        dllPath = '/home/s402/lerobot/one/lerobot-opi-main_gen72/lerobot-gen72/lerobot/common/robot_devices/robots/libRM_Base.so'
        self.pDll = ctypes.cdll.LoadLibrary(dllPath)
        #  连接机械臂
        self.pDll.RM_API_Init(72,0) 
        byteIP = bytes("192.168.1.18","gbk")
        self.nSocket = self.pDll.Arm_Socket_Start(byteIP, 8080, 200)
        print (self.nSocket)

        float_joint = ctypes.c_float*7
        self.left_joint_write = float_joint()
        self.right_joint_write = float_joint()

        #gen72API
        self.pDll.Movej_Cmd.argtypes = (ctypes.c_int, ctypes.c_float * 7, ctypes.c_byte, ctypes.c_float, ctypes.c_bool)
        self.pDll.Movej_Cmd.restype = ctypes.c_int
        self.pDll.Movej_CANFD.argtypes= (ctypes.c_int, ctypes.c_float * 7, ctypes.c_bool, ctypes.c_float)
        self.pDll.Movej_CANFD.restype = ctypes.c_int
        self.pDll.Get_Joint_Degree.argtypes = (ctypes.c_int, ctypes.c_float * 7)
        self.pDll.Get_Joint_Degree.restype = ctypes.c_int
        #self.pDll.Get_Gripper_State.argtypes = (ctypes.c_int, ctypes.POINTER(GripperState))
        self.pDll.Get_Gripper_State.restype = ctypes.c_int
        self.pDll.Set_Gripper_Position.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_bool, ctypes.c_int)
        self.pDll.Set_Gripper_Position.restype = ctypes.c_int
        self.pDll.Write_Single_Register.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,ctypes.c_bool)
        self.pDll.Write_Single_Register.restype = ctypes.c_int
        self.pDll.Set_Modbus_Mode.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_bool)
        self.pDll.Set_Modbus_Mode.restype = ctypes.c_int
        self.pDll.Set_Tool_Voltage.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_bool)
        self.pDll.Set_Tool_Voltage.restype = ctypes.c_int
        self.pDll.Close_Modbus_Mode.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_bool)
        self.pDll.Close_Modbus_Mode.restype = ctypes.c_int
        self.pDll.Get_Read_Holding_Registers.argtypes = (ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_int))
        self.pDll.Get_Read_Holding_Registers.restype=ctypes.c_int
        self.pDll.Set_High_Speed_Eth.argtypes = (ctypes.c_int, ctypes.c_byte, ctypes.c_bool)
        self.pDll.Set_High_Speed_Eth.restype = ctypes.c_int


        #-----------------------------------机械臂,夹爪,控制模式，modbus初始化
        float_joint = ctypes.c_float * 7
        joint_base = float_joint(*[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        ret=self.pDll.Movej_Cmd(self.nSocket, joint_base, 50, 1, 0)
        print('机械臂是否回到初始位置',ret)
        #打开高速网络配置
        self.pDll.Set_High_Speed_Eth(self.nSocket,1,0)
        #设置末端工具接口电压为24v
        self.pDll.Set_Tool_Voltage(self.nSocket,3,1)
        #打开modbus模式
        self.pDll.Set_Modbus_Mode(self.nSocket,1,115200,2,2,1)
        #初始化夹爪为打开状态
        self.pDll.Write_Single_Register(self.nSocket,1,40000,100,1,1)

    #---------------------------------------获取右机械臂和夹爪状态  
    def right_get(self,cmd: np.ndarray):
        self.right_joint=cmd[9:16]
        print("right_joint:",self.right_joint)
        self.right_gripper_two=cmd[16:18]
        print("right_gripper:",self.right_gripper)
        
    #---------------------------------------获取左机械臂和夹爪状态
    def left_get(self,cmd: np.ndarray):
        self.left_joint=cmd[0:7]
        print("left_joint:",self.left_joint)
        self.left_gripper_two=cmd[7:9]
        print("left_gripper:",self.left_gripper)

    #---------------------------------------夹爪数据处理
    def process_gripper(self):
        self.right_gripper=(self.right_gripper_two[0]/0.04)*100
        # self.left_gripper=(self.left_gripper_two[0]/0.04)*100

    #---------------------------------------夹爪阈值判断
    def threshold_gripprt(self,right_gipper:np.ndarray,left_gipper:np.ndarray):
        if (right_gipper <21 ) and (self.right_gipflag == 1):
            self.pDll.Write_Single_Register(self.nSocket, 1, 40000, 10 , 1, 1)
            self.right_gipflag=0
        #状态为闭合，且需要张开夹爪
        if (right_gipper>79) and (self.right_gipflag== 0):
            self.pDll.Write_Single_Register(self.nSocket, 1, 40000, 100, 1, 1)
            self.right_gipflag=1

        # if (left_gipper <21 ) and (self.left_gipflag == 1):
        #     self.pDll.Write_Single_Register(self.nSocket, 1, 40000, 10 , 1, 1)
        #     self.left_gipflag=0
        # #状态为闭合，且需要张开夹爪
        # if (left_gipper>79) and (self.left_gipflag== 0):
        #     self.pDll.Write_Single_Register(self.nSocket, 1, 40000, 100, 1, 1)
        #     self.left_gipflag=1

    #---------------------------------------运行
    def run(self,cmd: np.ndarray):
        self.right_get(cmd)
        # self.left_get(cmd)
        self.process_gripper()
        for i in range(7):
            self.right_joint_write[i] = self.right_joint[i]
        self.pDll.Movej_CANFD(self.nSocket,self.right_joint_write,FLAG_FOLLOW,0)
        # self.right_arm.rm_movej_canfd(self.right_joint, False)
        # self.left_arm.rm_movej_canfd(self.left_jopint, False)
        self.threshold_gripprt(self.right_gripperr)


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

    try:
        while True:
            if args.debug:
                cmd, latest = teleoperator.step()
            else:
                cmd = teleoperator.step()
                Gen_72("192.168.1.18").run(cmd)
                print("Non-debug mode cmd:", cmd)
    except KeyboardInterrupt:
        exit(0)

if __name__ == "__main__":
    main()

