import numpy as np
from avp_stream import VisionProStreamer as AVPStreamer
from typing import Tuple, List, Dict, Any

from .controller import ACEController

#获取左手和右手手腕的姿态和手指的姿态
def decode_msg(
    message: Dict[str, np.ndarray]
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    # 获取右手腕的姿势数据
    right_wrist_pose = message["right_wrist"][0]
    # 获取左手腕的姿势数据
    left_wrist_pose = message["left_wrist"][0]

    # 获取右手指尖的位置数据
    right_fingers = message["right_fingers"][:, 0:3, 3]
    # 获取左手指尖的位置数据
    left_fingers = message["left_fingers"][:, 0:3, 3]

    # 返回左手腕、右手腕、左手指尖和右手指尖的位置数据
    return left_wrist_pose, right_wrist_pose, left_fingers, right_fingers

class ACETeleop:
    def __init__(self, cfg: Dict[str, Any], ip: str, debug: bool = False):
        # 初始化AVPStreamer对象
        self.streamer = AVPStreamer(ip=ip)
        # 初始化ACEController对象
        self.controller = ACEController(cfg)

        # 设置调试模式
        self.debug = debug

        # 从配置中获取左臂的索引列表
        self.left_arm_indices: List[int] = cfg["left_arm_indices"]
        # 从配置中获取右臂的索引列表
        self.right_arm_indices: List[int] = cfg["right_arm_indices"]

    def step(self) -> Tuple[List[float], Any]:
        #获取服务端发送的左右手腕的姿态和手指的姿态
        data = decode_msg(self.streamer.latest)
        
        #处理，更新，并将计算得到的位置写入qpos
        self.controller.update(*data)

        #用cmd接收
        cmd = self.controller.qpos

        if self.debug:
            return cmd, self.streamer.latest.copy()
        else:
            return cmd
