import numpy as np
import pinocchio as pin
from numpy.linalg import norm, solve
from typing import List, Optional, Dict


class PinocchioMotionControl:

    def __init__(
        self,
        urdf_path: str,
        wrist_name: str,
        arm_init_qpos: np.ndarray,
        arm_config: Dict[str, float],
        arm_indices: Optional[List[int]] = [],
    ) -> None:
        self.arm_indices = arm_indices
        self.alpha = float(arm_config["out_lp_alpha"])

        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.dof = self.model.nq
        print("建模自由度---------------",self.dof)

        if arm_indices:
            locked_joint_ids = list(set(range(self.dof)) - set(self.arm_indices))
            locked_joint_ids = [
                id + 1 for id in locked_joint_ids
            ]  # account for universe joint
            self.model = pin.buildReducedModel(
                self.model, locked_joint_ids, np.zeros(self.dof)
            )
        self.arm_dof = self.model.nq

        self.lower_limit = self.model.lowerPositionLimit
        self.upper_limit = self.model.upperPositionLimit
        self.data: pin.Data = self.model.createData()

        self.wrist_id = self.model.getFrameId(wrist_name)

        # print(arm_init_qpos)在config中获得初始化位置
        self.qpos = arm_init_qpos
        pin.forwardKinematics(self.model, self.data, self.qpos)     #正解
        self.wrist_pose: pin.SE3 = pin.updateFramePlacement(
            self.model, self.data, self.wrist_id
        )

        self.damp = float(arm_config["damp"])
        self.ik_eps = float(arm_config["eps"])
        self.dt = float(arm_config["dt"])

    def control(self, target_pos: np.ndarray, target_rot: np.ndarray) -> np.ndarray:
        oMdes = pin.SE3(target_rot, target_pos)
        qpos = self.qpos.copy()

        ik_qpos = qpos.copy()
        ik_qpos = self.ik_clik(ik_qpos, oMdes, self.wrist_id)  # 解析后的逆解角度
        qpos = ik_qpos.copy()

        self.qpos = pin.interpolate(self.model, self.qpos, qpos, self.alpha)
        self.qpos = qpos.copy()

        return self.qpos.copy()

    def ik_clik(
        self, qpos: np.ndarray, oMdes: pin.SE3, wrist_id: int, iter: int = 1000
    ) -> np.ndarray:
        """
        迭代逆运动学求解函数

        参数:
        qpos (np.ndarray): 当前关节位置
        oMdes (pin.SE3): 目标末端执行器位姿
        wrist_id (int): 手腕关节的ID
        iter (int): 最大迭代次数，默认为1000

        返回:
        np.ndarray: 求解得到的关节位置
        """
        for _ in range(iter):
            # 计算当前关节位置的正运动学
            pin.forwardKinematics(self.model, self.data, qpos)
            # 获取当前手腕关节的位姿
            wrist_pose = pin.updateFramePlacement(self.model, self.data, wrist_id)
            # 计算当前位姿与目标位姿之间的逆变换
            iMd = wrist_pose.actInv(oMdes)
            # 计算误差向量
            err = pin.log(iMd).vector
            # 如果误差小于设定的阈值，则退出循环
            if norm(err) < self.ik_eps:
                break
            # 计算雅可比矩阵
            J = pin.computeFrameJacobian(self.model, self.data, qpos, wrist_id)
            # 计算误差雅可比矩阵
            J = -np.dot(pin.Jlog6(iMd.inverse()), J)
            # 计算速度
            v = -J.T.dot(solve(J.dot(J.T) + self.damp * np.eye(6), err))
            # 更新关节位置
            qpos = pin.integrate(self.model, qpos, v * self.dt)
        
        # 将关节位置限制在上下限范围内
        qpos = np.clip(qpos, self.lower_limit, self.upper_limit)
        return qpos
