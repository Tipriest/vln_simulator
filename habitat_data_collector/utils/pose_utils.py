"""
统一的姿态/位姿转换工具
约定:
- 右手系
- 欧拉角(roll, pitch, yaw), 旋转顺序 Z-Y-X:
  R = Rz(yaw) * Ry(pitch) * Rx(roll)
- 四元数接口形式 (w, x, y, z)
- 旋转矩阵 R: shape (3,3), v_world = R @ v_body
- 齐次变换 T: shape (4,4) = [R t; 0 1]
"""

import numpy as np
from scipy.spatial.transform import Rotation as R_  # 避免与变量名 R 冲突


# =============================
# R <-> 欧拉角
# =============================

def euler_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    欧拉角 -> 旋转矩阵
    输入单位: 弧度
    顺序: Z-Y-X (yaw, pitch, roll)
    """
    # SciPy: from_euler('zyx', [z, y, x]) = [yaw, pitch, roll]
    r = R_.from_euler('zyx', [yaw, pitch, roll])
    return r.as_matrix()  # (3,3)


def rot_to_euler(R: np.ndarray) -> np.ndarray:
    """
    旋转矩阵 -> 欧拉角
    返回: [roll, pitch, yaw] (弧度)
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    r = R_.from_matrix(R)
    yaw, pitch, roll = r.as_euler('zyx')  # 对应 Z-Y-X
    return np.array([roll, pitch, yaw])


# =============================
# R <-> 四元数
# =============================

def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """
    四元数 -> 旋转矩阵
    q: [w, x, y, z]
    返回: R (3,3)
    """
    q = np.asarray(q, dtype=float).flatten()
    w, x, y, z = q
    r = R_.from_quat([x, y, z, w])  # SciPy 使用 [x,y,z,w]
    return r.as_matrix()


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """
    旋转矩阵 -> 四元数
    返回: [w, x, y, z]
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    r = R_.from_matrix(R)
    x, y, z, w = r.as_quat()  # SciPy 返回 [x,y,z,w]
    return np.array([w, x, y, z])


# =============================
# 欧拉角 <-> 四元数
# =============================

def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    欧拉角 -> 四元数
    返回: [w, x, y, z]
    """
    R = euler_to_rot(roll, pitch, yaw)
    return rot_to_quat(R)


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """
    四元数 -> 欧拉角
    q: [w, x, y, z]
    返回: [roll, pitch, yaw]
    """
    R = quat_to_rot(q)
    return rot_to_euler(R)


# =============================
# 齐次变换 T <-> (R, t)
# =============================

def rt_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    (R, t) -> 齐次变换矩阵 T
    R: (3,3)
    t: (3,)
    返回: T (4,4) = [R t; 0 1]
    """
    R = np.asarray(R, dtype=float).reshape(3, 3)
    t = np.asarray(t, dtype=float).reshape(3,)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = t
    return T


def T_to_rt(T: np.ndarray):
    """
    齐次变换矩阵 T -> (R, t)
    返回: R(3,3), t(3,)
    """
    T = np.asarray(T, dtype=float).reshape(4, 4)
    R = T[:3, :3]
    t = T[:3, 3]
    return R, t


# =============================
# T <-> (欧拉角, t)
# =============================

def euler_t_to_T(roll: float, pitch: float, yaw: float,
                 tx: float, ty: float, tz: float) -> np.ndarray:
    """
    (欧拉角, t) -> T
    """
    R = euler_to_rot(roll, pitch, yaw)
    t = np.array([tx, ty, tz], dtype=float)
    return rt_to_T(R, t)


def T_to_euler_t(T: np.ndarray):
    """
    T -> (欧拉角, t)
    返回:
        roll, pitch, yaw, tx, ty, tz
    """
    R, t = T_to_rt(T)
    roll, pitch, yaw = rot_to_euler(R)
    tx, ty, tz = t
    return roll, pitch, yaw, tx, ty, tz


# =============================
# T <-> (四元数, t)
# =============================

def quat_t_to_T(q: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    (四元数, t) -> T
    q: [w, x, y, z]
    t: (3,)
    """
    R = quat_to_rot(q)
    return rt_to_T(R, t)


def T_to_quat_t(T: np.ndarray):
    """
    T -> (四元数, t)
    返回:
        q: [w, x, y, z], t: (3,)
    """
    R, t = T_to_rt(T)
    q = rot_to_quat(R)
    return q, t
