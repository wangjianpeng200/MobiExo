import numpy as np
from pytransform3d.rotations import quaternion_from_matrix
import socket

# def avp_to_mediapipe(fingers: np.ndarray) -> np.ndarray:
#     indices = np.array([0, 1, 2, 3, 4, 6, 7, 8, 9, 11, 12, 13, 14, 16, 17, 18, 19, 21, 22, 23, 24])
#     return fingers[indices]


class ACEInitializer:
    def __init__(self):
        """
        初始化ACEInitializer类的实例。

        该构造函数设置了初始化过程中所需的各种属性，包括左右手腕的历史数据列表、初始化状态、初始化进度和步长。

        属性:
        - left_wrist_list: 存储左手腕数据的列表。
        - right_wrist_list: 存储右手腕数据的列表。
        - initialized: 一个布尔值，指示初始化是否完成。
        - progress: 一个浮点数，表示初始化的进度，范围从0到1。
        - step: 一个浮点数，表示每次更新缓冲区时进度增加的步长。
        """
        # 初始化左手腕数据列表
        self.left_wrist_list = []
        # 初始化右手腕数据列表
        self.right_wrist_list = []
        # 初始化状态设置为False，表示尚未完成初始化
        self.initialized = False
        # 初始化进度设置为0，表示初始化尚未开始
        self.progress = 0
        # 每次更新缓冲区时进度增加的步长
        self.step = 0.02

    def init(
        self,
        left_wrist: np.ndarray,
        right_wrist: np.ndarray,
        left_fingers: np.ndarray,
        right_fingers: np.ndarray,
    ) -> bool:
        # right_fingers = avp_to_mediapipe(right_fingers)
        # left_fingers = avp_to_mediapipe(left_fingers)

        if not self.initialized:
            self._process(left_wrist, left_fingers, right_wrist, right_fingers)

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
    ) -> None:
        """
        处理手腕和关节数据以进行初始化。

        该方法检查当前的初始化进度，并根据进度和数据的连续性来更新缓冲区或重置初始化状态。

        参数:
        - left_wrist: 左手腕的姿态数据。
        - left_joints: 左手关节的姿态数据。
        - right_wrist: 右手腕的姿态数据。
        - right_joints: 右手关节的姿态数据。

        返回:
        无返回值。
        """
        # 如果初始化进度大于0，检查左手和右手的连续性
        if self.progress > 0:
            # 检查左手是否继续满足连续性条件
            left_continue = self._is_continue(
                left_wrist, left_joints, self.left_wrist_list[-1]
            )
            # 检查右手是否继续满足连续性条件
            right_continue = self._is_continue(
                right_wrist, right_joints, self.right_wrist_list[-1]
            )
            # 如果左手或右手不满足连续性条件，重置初始化状态
            if not (left_continue and right_continue):
                self._reset()

        # 更新缓冲区，将当前的手腕数据添加到列表中
        self._update_buffer(left_wrist, right_wrist)

        # 如果初始化进度达到或超过1，将初始化状态设置为True
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
        """
        检查手腕和关节数据是否满足连续性条件。

        该方法通过计算当前手腕位置与上一次手腕位置之间的距离、旋转角度变化以及关节角度的平坦度来判断是否满足连续性条件。

        参数:
        - wrist: 当前的手腕姿态数据。
        - joints: 当前的关节姿态数据。
        - last_wrist: 上一次的手腕姿态数据。

        返回:
        一个布尔值，表示是否满足连续性条件。
        """
        # 距离阈值，用于判断手腕位置是否变化过大
        dis_thresh = 0.02
        # 旋转角度阈值，用于判断手腕旋转是否过大
        rot_thresh = np.deg2rad(5)
        # 平坦度阈值，用于判断关节角度是否足够平坦
        flat_thresh = (0.01, np.deg2rad(15))

        # 计算当前手腕位置与上一次手腕位置之间的距离，并判断是否小于距离阈值
        not_far = np.linalg.norm(wrist[:3, 3] - last_wrist[:3, 3]) < dis_thresh
        # 计算当前手腕旋转角度与上一次手腕旋转角度之间的变化，并判断是否小于旋转角度阈值
        not_rotated = (
            self._rotation_change(wrist[:3, :3], last_wrist[:3, :3]) < rot_thresh
        )

        # 计算关节角度，并判断是否在平坦度阈值范围内
        angles = self._joint_angles(joints)
        flat = flat_thresh[0] < np.mean(angles) < flat_thresh[1]

        # 返回是否满足连续性条件的结果
        return not_far and not_rotated and flat

    @staticmethod
    def _joint_angles(joints: np.ndarray) -> np.ndarray:
        """
        计算手指关节角度。

        该方法计算每个手指的关节角度，即从手指尖到手掌中心的向量与从手掌中心到手指根的向量之间的夹角。

        参数:
        - joints: 包含手指关节位置的数组。

        返回:
        一个包含每个手指关节角度的数组。
        """
        # 手指尖的索引
        tip_indices = np.array([4, 8, 12, 16, 20])
        # 手掌中心的索引
        palm_indices = np.array([1, 5, 9, 13, 17])

        # 根关节的位置
        root = joints[0:1, :]
        # 手指尖的位置
        tips = joints[tip_indices]
        # 手掌中心的位置
        palm_bones = joints[palm_indices]

        # 计算从手指尖到根关节的单位向量
        tip_vec = (tips - root) / np.linalg.norm(tips - root, axis=1, keepdims=True)
        # 计算从手掌中心到根关节的单位向量
        palm_vec = (palm_bones - root) / np.linalg.norm(
            palm_bones - root, axis=1, keepdims=True
        )

        # 计算两个向量之间的夹角
        angles = np.arccos(np.clip(np.sum(tip_vec * palm_vec, axis=1), -1.0, 1.0))
        return angles

    def _reset(self) -> None:
        self.left_wrist_list.clear()
        self.right_wrist_list.clear()
        self.progress = 0

    def _update_buffer(self, left_wrist: np.ndarray, right_wrist: np.ndarray) -> None:
        self.progress += self.step
        self.left_wrist_list.append(left_wrist)
        self.right_wrist_list.append(right_wrist)
