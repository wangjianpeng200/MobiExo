from pathlib import Path
from typing import List, Dict
import numpy as np
from pytransform3d.rotations import quaternion_from_matrix, matrix_from_quaternion

import ace_teleop
from ace_teleop.control.motion_control import PinocchioMotionControl
from ace_teleop.control.filters import LPFilter, LPRotationFilter
from dex_retargeting.retargeting_config import RetargetingConfig

def wrist_filters(
    wrist_mat: np.ndarray, pos_filter: LPFilter, rot_filter: LPRotationFilter
) -> np.ndarray:
    filtered_mat = wrist_mat.copy()#创建一个新的矩阵，其大小和wrist_mat相同
    filtered_mat[:3, 3] = pos_filter.next(wrist_mat[:3, 3])
    filtered_mat[:3, :3] = matrix_from_quaternion(
        rot_filter.next(quaternion_from_matrix(wrist_mat[:3, :3]))
    )
    return filtered_mat

def categorize_and_map_value(
    array: np.ndarray,
    value_range: List[float],
    min_distance: float = 0.05,
    max_distance: float = 0.08,
) -> float:
    distance = np.linalg.norm(array[0] - array[1])
    clamped_distance = max(min_distance, min(distance, max_distance))
    mapped_value = (clamped_distance - min_distance) * (
        (value_range[1] - value_range[0]) / (max_distance - min_distance)
    ) + value_range[0]
    return mapped_value

class ACEController:
    def __init__(self, cfg: dict = None) -> None:
        if cfg is None:
            raise ValueError("Need to provide a config file.")

        self.default_urdf_dir = Path(ace_teleop.__path__[0]) / "assets"

        self.dof = cfg["dof_num"]
        self._qpos = np.zeros(self.dof)
        self.cfg = cfg
        
        self._init_indices()
        self._init_controllers()

        self._init_filters()

    def _init_indices(self) -> None:
        self.human_hand_indices = self.cfg["human_hand_indices"]
        self.left_arm_indices = self.cfg["left_arm_indices"]
        self.right_arm_indices = self.cfg["right_arm_indices"]
        self.left_ee_indices = self.cfg["left_ee_indices"]
        self.right_ee_indices = self.cfg["right_ee_indices"]

    def _init_controllers(self) -> None:
        self.ee_type = self.cfg["ee_type"]
        urdf_path = self.default_urdf_dir / Path(self.cfg["urdf_path"])
        arm_config = self.cfg["arm"]

        self.left_ee_controller = PinocchioMotionControl(
            urdf_path,
            self.cfg["left_ee"],
            np.array(self.cfg['left_arm_init']),
            arm_config,
            arm_indices=self.left_arm_indices,
        )
        self.right_ee_controller = PinocchioMotionControl(
            urdf_path,
            self.cfg["right_ee"],
            np.array(self.cfg['right_arm_init']),
            arm_config,
            arm_indices=self.right_arm_indices,
        )

        if self.ee_type == "hand":
            self.hand_indices = self.cfg["hand_indices"]
            self._init_hand_controllers()
        elif self.ee_type == "gripper":
            self._init_gripper()

        self.configured = True

    def _init_filters(self) -> None:
        wrist_alpha = self.cfg["wrist_low_pass_alpha"]
        ee_alpha = self.cfg["hand_low_pass_alpha"]

        self.left_wrist_pos_filter = LPFilter(wrist_alpha)
        self.left_wrist_rot_filter = LPRotationFilter(wrist_alpha)
        self.right_wrist_pos_filter = LPFilter(wrist_alpha)
        self.right_wrist_rot_filter = LPRotationFilter(wrist_alpha)

        self.left_fingertip_pos_filter = LPFilter(ee_alpha)
        self.right_fingertip_pos_filter = LPFilter(ee_alpha)

    def _init_hand_controllers(self) -> None:
        ee_config = self.cfg["ee"]

        RetargetingConfig.set_default_urdf_dir(self.default_urdf_dir)

        left_config = RetargetingConfig.from_dict(ee_config["left_ee"])
        self.left_retargeting = left_config.build()

        right_config = RetargetingConfig.from_dict(ee_config["right_ee"])
        self.right_retargeting = right_config.build()

    def _init_gripper(self) -> None:
        ee_config = self.cfg["ee"]

        self.left_gripper_range = ee_config["left_ee"]["gripper_range"]
        self.right_gripper_range = ee_config["right_ee"]["gripper_range"]
        self.gripper_type = self.cfg["gripper_type"]

    def update(
        self,
        left_wrist: np.ndarray,
        right_wrist: np.ndarray,
        left_fingertip_pos: np.ndarray,
        right_fingertip_pos: np.ndarray,
    ) -> None:
        if not self.configured:
            raise ValueError("ACE controller has not been configured.")
        
        #使用wrist_filters函数对左右手腕的位置和旋转进行滤波处理。这里假设wrist_filters是一个自定义函数，
        #它接受手腕的当前状态（位置和方向）以及两个滤波器作为参数，并返回滤波后的手腕状态。
        left_wrist = wrist_filters(
            left_wrist, self.left_wrist_pos_filter, self.left_wrist_rot_filter
        )
        # 打印 left_wrist 矩阵
        # print("Left Wrist Transformation Matrix:")
        # print("滤波后的手腕信息",left_wrist)
        right_wrist = wrist_filters(
            right_wrist, self.right_wrist_pos_filter, self.right_wrist_rot_filter
        )
        
        #接收目标位置和目标旋转并返回更新后的位置数组
        left_arm_qpos = self.left_ee_controller.control(
            left_wrist[:3, 3], left_wrist[:3, :3]
        )
        right_arm_qpos = self.right_ee_controller.control(
            right_wrist[:3, 3], right_wrist[:3, :3]
        )

        # 打印 left_arm_qpos 数组
        # print("Left Arm Joint Positions (qpos):")
        # print("滤波后的手腕信息",left_arm_qpos)

        left_fingertip_pos = self.left_fingertip_pos_filter.next(left_fingertip_pos)
        right_fingertip_pos = self.right_fingertip_pos_filter.next(right_fingertip_pos)
        
        #根据self.ee_type（末端执行器类型，可以是"hand"或"gripper"），采取不同的处理方式
        if self.ee_type == "hand":
            left_ee_qpos = self.left_retargeting.retarget(
                left_fingertip_pos[self.human_hand_indices]
            )[self.hand_indices]
            right_ee_qpos = self.right_retargeting.retarget(
                right_fingertip_pos[self.human_hand_indices]
            )[self.hand_indices]
        elif self.ee_type == "gripper":
            left_ee_qpos = categorize_and_map_value(
                left_fingertip_pos[self.human_hand_indices], self.left_gripper_range
            )
            right_ee_qpos = categorize_and_map_value(
                right_fingertip_pos[self.human_hand_indices], self.right_gripper_range
            )
            if self.gripper_type == "2dof":
                left_ee_qpos = np.array([left_ee_qpos, left_ee_qpos])
                right_ee_qpos = np.array([right_ee_qpos, right_ee_qpos])

        self._qpos[self.left_arm_indices] = left_arm_qpos
        self._qpos[self.right_arm_indices] = right_arm_qpos
        self._qpos[self.left_ee_indices] = left_ee_qpos
        self._qpos[self.right_ee_indices] = right_ee_qpos

    @property
    def qpos(self) -> np.ndarray:
        return self._qpos.astype(np.float32).copy()
