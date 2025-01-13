import numpy as np
import transforms3d.euler as euler

from typing import Tuple
from pyquaternion import Quaternion
from dataclasses import dataclass, field


def get_mapping_offset(center, ee_pos, action_scale):
    # print("center: ", center, "ee_pos: ", ee_pos, "action_scale: ", action_scale)
    offset = np.array(center) - np.array(ee_pos) * action_scale
    return offset


def smooth_rot(init_matrix, target_matrix, alpha):
    init_quat = Quaternion(matrix=init_matrix)
    target_quat = Quaternion(matrix=target_matrix)
    slerp_quat = Quaternion.slerp(init_quat, target_quat, alpha)
    return slerp_quat.rotation_matrix


def smooth_pos(init_pos, target_pos, alpha):
    # Ensure inputs are numpy arrays
    init_pos = np.array(init_pos)
    target_pos = np.array(target_pos)

    # Interpolate position vectors linearly
    interp_pos = (1 - alpha) * init_pos + alpha * target_pos

    return interp_pos


@dataclass
class R_map:
    mapping_rot: np.ndarray
    init_rot: np.ndarray

    roll_scale: float = 1
    pitch_scale: float = 1
    yaw_scale: float = 1

    roll_limit: np.ndarray = field(default_factory=lambda: np.array([-1.57, 1.57]))
    pitch_limit: np.ndarray = field(default_factory=lambda: np.array([-1.57, 1.57]))
    yaw_limit: np.ndarray = field(default_factory=lambda: np.array([-1.57, 1.57]))

    roll_offset: float = 0


def euler_to_matrix(roll, pitch, yaw):

    # 将欧拉角从度数转换为弧度
    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)

    # 使用scipy库中的euler2mat函数将欧拉角转换为旋转矩阵
    # 参数axes="sxyz"表示按照Z-Y-X的顺序进行旋转
    R_matrix = euler.euler2mat(roll, pitch, yaw, axes="sxyz")
    return R_matrix


def clamp_point_to_sphere(
    point: Tuple[float, float, float], center: Tuple[float, float, float], radius: float
) -> Tuple[float, float, float]:
    """
    将一个点限制在一个球体的表面上。如果点在球体内，则返回该点本身；如果点在球体外，则将该点投影到球体表面上并返回。

    参数:
    point (Tuple[float, float, float]): 要限制的点的坐标。
    center (Tuple[float, float, float]): 球体的中心坐标。
    radius (float): 球体的半径。

    返回:
    Tuple[float, float, float]: 限制后的点的坐标。
    """
    # 将输入的点和中心坐标转换为numpy数组
    point = np.array(point)
    center = np.array(center)

    # 计算点到球心的向量
    vector = point - center

    # 计算点到球心的距离
    distance = np.linalg.norm(vector)

    # 如果点在球体内，则返回该点本身
    if distance <= radius:
        return tuple(point)
    # 如果点在球体外，则将该点投影到球体表面上并返回
    else:
        unit_vector = vector / distance
        clamped_point = center + unit_vector * radius
        return tuple(clamped_point)
