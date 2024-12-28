from concurrent import futures
import grpc
import numpy as np

from ace_teleop.configs.server.ace_const import *
from ace_teleop.stream.streamer import HandTrackingServicer, handtracking_pb2_grpc
from ace_teleop.server.utils import *

class Server:
    def __init__(self, cfg: dict) -> None:
        
        self.mode = cfg["mode"]

        agent_cfg = {}
        self.enable_agent = {}
        self.wrist = {}
        for name in ["left", "right"]:
            agent_cfg[name] = cfg.get(f"{name}_agent", None)

            self.enable_agent[name] = agent_cfg[name] is not None

            if not self.enable_agent[name]:
                self.initialized = True
                print("Single agent mode do not support auto initialization")

            self.wrist[name] = None

        if not self.enable_agent["left"] and not self.enable_agent["right"]:
            raise ValueError("No agent is enabled")

        self.cfg = cfg
        self.init_cfg()
        self.init_server()

    def init_cfg(self) -> None:

        self.is_ACE = self.cfg["is_ACE"]

        self.pos_scale = self.cfg["pos_scale"]
        self.roll_scale = self.cfg["roll_scale"]
        self.pitch_scale = self.cfg["pitch_scale"]
        self.yaw_scale = self.cfg["yaw_scale"]

        self.roll_limit = self.cfg["roll_limit"]
        self.pitch_limit = self.cfg["pitch_limit"]
        self.yaw_limit = self.cfg["yaw_limit"]

        self.roll_offset = self.cfg.get("roll_offset", 0)

        self.wrist_init_rot, self.wrist_init_pos = {}, {}
        self.center_l, self.radius_l = {}, {}
        for name in ["left", "right"]:
            if self.enable_agent[name]:
                wrist_cfg = self.cfg[f"{name}_wrist"]
                self.wrist_init_rot[name] = wrist_cfg[f"{name}_wrist_init_rot"]
                self.wrist_init_pos[name] = wrist_cfg[f"{name}_wrist_init_pos"]
                self.center_l[name] = wrist_cfg[f"{name}_center"]    # 球心坐标
                self.radius_l[name] = wrist_cfg[f"{name}_radius"]    # 球半径
    
    def init_server(self) -> None:
        """
        初始化gRPC服务器。

        该方法创建一个gRPC服务器实例，将HandTrackingServicer添加到服务器中，并启动服务器。

        参数:
        无

        返回:
        无
        """
        # 创建一个gRPC服务器实例，使用线程池执行器，最大工作线程数为10
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        # 创建一个HandTrackingServicer实例
        self.servicer = HandTrackingServicer()
        # 将HandTrackingServicer添加到服务器中
        handtracking_pb2_grpc.add_HandTrackingServiceServicer_to_server(
            self.servicer, self.server
        )
        # 为服务器添加一个不安全的端口，监听所有可用的网络接口，端口号为12345
        self.server.add_insecure_port("[::]:12345")

        # 启动服务器
        self.server.start()

        # 将头部的坐标系从YUP转换为ZUP，并存储在servicer的head属性中
        self.servicer.head = np.dot(YUP2ZUP_INV_2D, HEAD)
        # 将右侧关键点的默认值存储在servicer的points_right属性中
        self.servicer.points_right[:] = default_keypoint
        # 将左侧关键点的默认值存储在servicer的points_left属性中
        self.servicer.points_left[:] = default_keypoint
        # 将右侧的变换矩阵初始化为单位矩阵，并存储在servicer的matrix_right属性中
        self.servicer.matrix_right = np.eye(4)
        # 将左侧的变换矩阵初始化为单位矩阵，并存储在servicer的matrix_left属性中
        self.servicer.matrix_left = np.eye(4)

    def run(self) -> None:
        while True:
            self.servicer.update_event.set()