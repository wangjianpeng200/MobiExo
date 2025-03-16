import numpy as np
from pytransform3d.rotations import quaternion_from_matrix
import socket

# def avp_to_mediapipe(fingers: np.ndarray) -> np.ndarray:
#     indices = np.array([0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24])
#     return fingers[indices]


class ACEInitializer:
    def __init__(self):
        self.left_wrist_list = []
        self.right_wrist_list = []
        self.initialized = False
        self.progress = 0
        self.step = 0.02

    def init(
        self,
        left_wrist: np.ndarray,
        right_wrist: np.ndarray,
        left_fingers: np.ndarray,
        right_fingers: np.ndarray,
        left_gripper: float,
        right_gripper: float,
    ) -> bool:
        # right_fingers = avp_to_mediapipe(right_fingers)
        # left_fingers = avp_to_mediapipe(left_fingers)

        #如果没有初始化则开始初始化
        if not self.initialized:
        #调用初始化的函数
            self._process(left_wrist, left_fingers, right_wrist, right_fingers, left_gripper, right_gripper)

        print(f"Initialization progress: {self.progress * 100:.2f}%")
        return self.initialized
    
    def send_start_message(self):
        # 创建一个 UDP 套接字
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 定义本地地址和端口
        local_address = ("127.0.0.1", 12345)
        # 发送 "start" 消息
        message = "start".encode('utf-8')
        udp_socket.sendto(message, local_address)
        # 关闭套接字
        udp_socket.close()

    def _process(
        self,
        left_wrist: np.ndarray,
        left_joints: np.ndarray,
        right_wrist: np.ndarray,
        right_joints: np.ndarray,
        left_value: float,
        right_value: float,
    ) -> None:
        if self.progress > 0:
            if not (left_value<1 and right_value<1):
                self._reset()
                self._reset()

        self._update_buffer(left_wrist, right_wrist)

        if self.progress >= 1:
            self.initialized = True
            self.send_start_message()

    def _rotation_change(self, R1: np.ndarray, R2: np.ndarray) -> float:
        relative_R = np.dot(R2, R1.T)
        q = quaternion_from_matrix(relative_R)
        angle = 2 * np.arccos(np.clip(q[0], -1.0, 1.0))
        return angle

    def _is_continue(
        self, wrist: np.ndarray, joints: np.ndarray, last_wrist: np.ndarray
    ) -> bool:
        dis_thresh = 0.02
        rot_thresh = np.deg2rad(5)
        flat_thresh = (0.01, np.deg2rad(15))

        not_far = np.linalg.norm(wrist[:3, 3] - last_wrist[:3, 3]) < dis_thresh
        not_rotated = (
            self._rotation_change(wrist[:3, :3], last_wrist[:3, :3]) < rot_thresh
        )

        angles = self._joint_angles(joints)
        flat = flat_thresh[0] < np.mean(angles) < flat_thresh[1]
        return not_far and not_rotated and flat

    @staticmethod
    def _joint_angles(joints: np.ndarray) -> np.ndarray:
        tip_indices = np.array([4, 8, 12, 16, 20])
        palm_indices = np.array([1, 5, 9, 13, 17])

        root = joints[0:1, :]
        tips = joints[tip_indices]
        palm_bones = joints[palm_indices]
        tip_vec = (tips - root) / np.linalg.norm(tips - root, axis=1, keepdims=True)
        palm_vec = (palm_bones - root) / np.linalg.norm(
            palm_bones - root, axis=1, keepdims=True
        )
        angles = np.arccos(np.clip(np.sum(tip_vec * palm_vec, axis=1), -1.0, 1.0))
        return angles

    def _reset(self) -> None:
        self.left_wrist_list.clear()
        self.right_wrist_list.clear()
        self.progress = 0

    def _update_buffer(self, left_wrist: np.ndarray, right_wrist: np.ndarray) -> None:
        self.progress += self.step
        # self.left_wrist_list.append(left_wrist)
        # self.right_wrist_list.append(right_wrist)
