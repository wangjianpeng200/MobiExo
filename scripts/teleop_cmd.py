from pathlib import Path
import argparse
import yaml
import numpy as np
from typing import Dict, Any

import ace_teleop
from ace_teleop.control.teleop import ACETeleop
from Robotic_Arm.rm_robot_interface import *


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
        # 创建机械臂对象,三线程
        self.right_arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)# 创建机械臂对象,三线程
        # arm_left = RoboticArm()
        right_handle = self.right_arm.rm_create_robot_arm(self.ip, 8080)# 创建机械臂连接，打印连接id
        # left_handl= self.left_arm.rm_create_robot_arm(self.ip, 8080)
        print(right_handle.id)
        self.right_arm.rm_set_arm_run_mode(0)#右臂设置为仿真模式

        #-----------------------------------机械臂,夹爪,控制模式，modbus初始化
        self.right_arm.rm_set_tool_voltage(3) #末端工具电压设置为24v
        # self.left_arm.rm_set_tool_voltage(3) 
        base_flag=self.right_arm.rm_movej(self.left_basejoint, 50, 0, 0, 0)    # 左臂运动到初始位置
        if base_flag!=0:
            print("机械臂运动到初始位置wrong!!!!!,base_flag:",base_flag)
        # self.left_arm.rm_movej(self.left_basejoint, 20, 0, 0, 0)

        self.right_arm.rm_set_modbus_mode(1,115200,2)   #modbus初始化
        self.write_params = rm_peripheral_read_write_params_t(1, 40000, 1)
        rm_peripheral_read_write_params_t(self.write_params,100) #设置modbus写入参数，打开夹爪

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
            rm_peripheral_read_write_params_t(self.write_params,10)
            self.right_gipflag=0
        #状态为闭合，且需要张开夹爪
        if (right_gipper>79) and (self.right_gipflag== 0):
            rm_peripheral_read_write_params_t(self.write_params,100)
            self.right_gipflag=1

        # if (left_gipper <21 ) and (self.left_gipflag == 1):
        #     rm_peripheral_read_write_params_t(self.write_params,10)
        #     self.left_gipflag=0
        # #状态为闭合，且需要张开夹爪
        # if (left_gipper>79) and (self.left_gipflag== 0):
        #     rm_peripheral_read_write_params_t(self.write_params,100)
        #     self.left_gipflag=1

    #---------------------------------------运行
    def run(self,cmd: np.ndarray):
        self.right_get(cmd)
        # self.left_get(cmd)
        self.process_gripper()
        self.right_arm.rm_movej_canfd(self.right_joint, False)
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
                print("Non-debug mode cmd:", cmd)
    except KeyboardInterrupt:
        exit(0)

if __name__ == "__main__":
    main()

