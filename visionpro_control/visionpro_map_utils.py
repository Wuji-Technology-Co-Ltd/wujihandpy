import numpy as np
import json
import os
from dataclasses import dataclass

@dataclass(frozen=True)
class MapCfg:
    min: float | None = None   # 角度下限（度）
    max: float | None = None   # 角度上限（度）
    reverse: bool = False      # 是否反向
    fixed: int | None = None   # 固定原始输出
    deadzone: float = 0.0   
    gamma: float = 1.0

# ---- 2) 内嵌配置表（按手指 x 关节）----
# 关节顺序：0=flexion, 1=abduction, 2=joint2, 3=joint3
CFG: dict[str, tuple[MapCfg, MapCfg, MapCfg, MapCfg]] = {
    "thumb": (
        MapCfg(min=8, max=25),
        MapCfg(min=18,max=42,  reverse=True),
        MapCfg(min=5, max=60),
        MapCfg(min=5, max=65),
    ),
    "index": (
        MapCfg(min=-15, max=70),
        MapCfg(min=150, max=168),
        MapCfg(min=-10, max=100),
        MapCfg(min=0, max=50),
    ),
    "middle": (
        MapCfg(min=-20, max=65),
        MapCfg(min=165, max=180),                  
        MapCfg(min=-5, max=100),
        MapCfg(min=-5, max=70),
    ),
    "ring": (
        MapCfg(min=-20, max=80),
        MapCfg(min=165, max=175,reverse=True),
        MapCfg(min=-5, max=100),
        MapCfg(min=-5, max=75),
    ),
    "pinky": (
        MapCfg(min=-20, max=95),
        MapCfg(min=145, max=160,reverse=True),
        MapCfg(min=-10, max=85),
        MapCfg(min=-10, max=80),
    ),
}

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
JOINTS  = ["joint0", "joint1", "joint2", "joint3"]

_stats_file = "/home/uji/Robben_ws/ROS_ws/src/wujihand_ros2/wujihand_ros2/hand_angles_stats.json"

def record_angle_stats(control_angles, path: str = _stats_file):
    """
    记录每个 finger/joint 的 min/max，实时更新保存到 JSON 文件。
    control_angles: (5,4) numpy array, 单位度
    """
    # 初始化文件
    if not os.path.exists(path):
        stats = {f: {f"joint{j}": {"min": 1e9, "max": -1e9}
                     for j in range(4)}
                 for f in FINGERS}
    else:
        with open(path, "r", encoding="utf-8") as f:
            stats = json.load(f)

    # 更新
    for i, finger in enumerate(FINGERS):
        for j in range(4):
            val = float(control_angles[i, j])
            stats[finger][f"joint{j}"]["min"] = min(stats[finger][f"joint{j}"]["min"], val)
            stats[finger][f"joint{j}"]["max"] = max(stats[finger][f"joint{j}"]["max"], val)

    # 保存
    with open(path, "w", encoding="utf-8") as f:
        json.dump(stats, f, indent=2, ensure_ascii=False)

class LowPassEMA:
    def __init__(self, alpha=0.18, x0=None):
        self.alpha = alpha
        self.y = None if x0 is None else np.array(x0, dtype=float)

    def step(self, x):
        x = np.array(x, dtype=float)
        if self.y is None:
            self.y = x.copy()
        self.y += self.alpha * (x - self.y)
        return self.y

def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """L2 normalize; raise if near-zero."""
    n = np.linalg.norm(v)
    if n < eps:
        raise ValueError("zero-length")
    return v / n

def calc_finger_abduction_angle(A: np.ndarray, B: np.ndarray, 
                                C: np.ndarray, D: np.ndarray, eps: float = 1e-12) -> float:
    """
    在掌面 ABC 内，以 C 为角点，计算 ∠(CA, CE) 的带符号角度。
    E 为 D 在平面 ABC 上的投影点。
    返回值范围 [-180, 180] 度。
    """
    # 掌面法向
    n_palm = np.cross(B - A, C - A)
    n_norm = np.linalg.norm(n_palm)
    if n_norm < eps:
        raise ValueError("Palm plane degenerate: A,B,C nearly collinear.")
    n_palm /= n_norm

    # 投影 D -> E
    E = D - np.dot(D - C, n_palm) * n_palm

    # 向量 CA, CE
    v_CA = A - C
    v_CE = E - C
    if np.linalg.norm(v_CA) < eps or np.linalg.norm(v_CE) < eps:
        return 180
    ref = v_CA / np.linalg.norm(v_CA)
    tgt = v_CE / np.linalg.norm(v_CE)

    # 带符号角（弧度转角度）
    cos_th = float(np.clip(np.dot(ref, tgt), -1.0, 1.0))
    sin_th = float(np.dot(n_palm, np.cross(ref, tgt)))
    angle_rad = float(np.arctan2(sin_th, cos_th))
    return np.degrees(angle_rad)  # 转换为角度

def compute_thumb_joint0_angle(
    A: np.ndarray,  # wrist
    B: np.ndarray,  # 掌面参考点1（如 index MCP）
    C: np.ndarray,  # 掌面参考点2（如 pinky MCP）
    D: np.ndarray,  # 被测点（拇指 IP/TIP）
    eps: float = 1e-12
) -> float:
    """
    计算相对掌面 ABC 的"抬升角"（度）：
    - 掌面法向 n = (B-A)×(C-A) 归一化
    - v = A->D
    - elevation = atan2(|v 在法向上的分量|, |v 在掌面内的分量|)
    返回度数。
    """
    # 掌面法向
    n = np.cross(B - A, C - A)
    n_norm = np.linalg.norm(n)
    if n_norm < eps:
        raise ValueError("Palm plane degenerate: A,B,C nearly collinear.")
    n = n / n_norm

    # A->D 向量
    v = D - A
    v_norm = np.linalg.norm(v)
    if v_norm < eps:
        return 0.0

    # 分解到"法向/切向"
    v_n = abs(float(np.dot(v, n)))                # 法向分量长度
    v_t = float(np.linalg.norm(v - v_n * n))      # 平面内分量长度

    # 抬升角：在 [0, 90] 内更稳定的写法
    elev_rad = np.arctan2(v_n, v_t)               # 避免精度问题，优于 asin
    return float(np.degrees(elev_rad))

def compute_thumb_joint1_angle(
    A: np.ndarray,  # wrist
    B: np.ndarray,  # middle MCP
    C: np.ndarray,  # thumb MCP
    eps: float = 1e-12
) -> float:
    """
    计算腕点 A 处两向量 AB 与 AC 的无符号夹角（拇指外展量），结果范围 [0, 180] 度。
    A = wrist, B = middle MCP, C = thumb MCP
    """
    v1 = B - A   # wrist -> middle
    v2 = C - A   # wrist -> thumb
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < eps or n2 < eps:
        raise ValueError("Degenerate vectors: wrist==middle or wrist==thumb")

    # θ = atan2(‖v1×v2‖, v1·v2)，数值稳定
    cross_norm = np.linalg.norm(np.cross(v1, v2))
    dot_val    = float(np.dot(v1, v2))
    theta_rad  = float(np.arctan2(cross_norm, dot_val))  # [0, π]
    theta_deg  = np.degrees(theta_rad)
    return theta_deg

def extract_rotation_matrix(matrix):
    return matrix[:3, :3]

def calculate_angle_between_matrices(mat1, mat2):
    R_rel = np.dot(mat1.T, mat2)
    trace = np.trace(R_rel)
    angle_rad = np.arccos(np.clip((trace - 1) / 2.0, -1.0, 1.0))
    return np.degrees(angle_rad)  # 转换为角度

def solve_thumb_angles(finger_matrices):
    thumb_mats = finger_matrices[:4]

    flexion_angles = []
    for i in range(3):
        R1 = extract_rotation_matrix(thumb_mats[i])
        R2 = extract_rotation_matrix(thumb_mats[i + 1])
        flexion_angles.append(calculate_angle_between_matrices(R1, R2))

    # --- 组装结果 ---
    angles = [0.0, 0.0, 0.0, 0.0]
    angles[2] = flexion_angles[1]
    angles[3] = flexion_angles[2]
    return angles

# 添加全局角度历史记录
_angle_history = {}

def solve_joint_angle_with_unwrapping(matrix, joint_id=None):
    """
    Solve joint angle from a 4x4 transformation matrix with angle unwrapping
    Args:
        matrix: 4x4 numpy array or matrix  
        joint_id: 关节ID，用于角度解包
    Returns:
        float: joint angle in degrees
    """
    # Ensure input is numpy array
    if not isinstance(matrix, np.ndarray):
        matrix = np.array(matrix)
    # Check matrix dimensions
    if matrix.shape != (4, 4):
        raise ValueError("Input matrix must be 4x4.")
    # Calculate angle using atan2； rotation around z axis
    m10 = matrix[1, 0]  # matrix.m10()
    m00 = matrix[0, 0]  # matrix.m00()
    joint_angle_rad = np.arctan2(m10, m00)
    joint_angle_deg = np.degrees(joint_angle_rad)
    
    # 角度解包：处理 -180° 到 180° 的跳跃
    if joint_id is not None and joint_id in _angle_history:
        prev_angle = _angle_history[joint_id]
        angle_diff = joint_angle_deg - prev_angle
        
        # 如果角度差超过 180°，说明发生了跳跃
        if angle_diff > 180:
            joint_angle_deg -= 360
        elif angle_diff < -180:
            joint_angle_deg += 360
    
    # 更新历史记录
    if joint_id is not None:
        _angle_history[joint_id] = joint_angle_deg
    
    return joint_angle_deg

def solve_finger_angles(finger_matrices, finger_name):
    """
    Solve finger joint angles from finger matrices with unwrapping
    Args:
        finger_matrices: list of 4x4 matrices for finger joints
        finger_name: 手指名称，用于角度解包
    Returns:
        list: finger joint angles
    """

    angles = []
    for i, finger_matrix in enumerate(finger_matrices[:4]):
        joint_id = f"{finger_name}_joint{i}"
        angle = solve_joint_angle_with_unwrapping(finger_matrix, joint_id)
        angles.append(angle)

    # Calculate relative angles (difference between consecutive joints)
    for i in range(3, 0, -1):  # 3, 2, 1 (reverse order)
        angles[i] -= angles[i - 1]
        
        # Normalize angles to [-180, 180] range
        if angles[i] > 180:
            angles[i] -= 360
        elif angles[i] < -180:
            angles[i] += 360
    
    # Set first angle equal to second, second angle to 0
    angles[0] = angles[1]
    angles[1] = 360
    return angles

def to_unsigned_angle(angle_deg: float) -> float:
    """
    把 [-180,180] 的角度转成 [0,180]：
    - 正数保持不变
    - 负数变成 180 - abs(angle)
    """
    if angle_deg < 0:
        return 360 + angle_deg
    return angle_deg

def solve_hand_angles(hand_matrices):
    """
    Solve hand joint angles from hand matrices
    Args:
        hand_matrices: list of 4x4 matrices for hand joints (25 matrices)
    Returns:
        numpy.ndarray: 5x4 matrix of hand joint angles in degrees
    """
    # Initialize 5x4 result matrix
    result = np.zeros((5, 4))
    
    # Solve angles for each finger (starting from index 5)
    # First row (index 0) stays as zeros (hack)
    result[0] = solve_thumb_angles(hand_matrices[:5])    # Thumb finger
    result[1] = solve_finger_angles(hand_matrices[5:10], "index")    # Index finger
    result[2] = solve_finger_angles(hand_matrices[10:15], "middle")   # Middle finger  
    result[3] = solve_finger_angles(hand_matrices[15:20], "ring")   # Ring finger
    result[4] = solve_finger_angles(hand_matrices[20:25], "pinky")   # Pinky finger

    result[0, 0] = compute_thumb_joint0_angle(hand_matrices[0][:3, 3],hand_matrices[5][:3, 3], hand_matrices[20][:3, 3],hand_matrices[2][:3, 3])
    # #拇指的开合使用与中指的角度计算，(通过电机1的运动，几何上推断)
    result[0,1] = compute_thumb_joint1_angle(hand_matrices[0][:3, 3], hand_matrices[10][:3, 3],hand_matrices[3][:3, 3])
    # 计算并输出 calc_finger_abduction_angle 的值
    result[1,1] = to_unsigned_angle(calc_finger_abduction_angle(hand_matrices[0][:3, 3],hand_matrices[10][:3, 3], hand_matrices[5][:3, 3],hand_matrices[7][:3, 3]))
    result[2,1] = to_unsigned_angle(calc_finger_abduction_angle(hand_matrices[0][:3, 3],hand_matrices[15][:3, 3], hand_matrices[10][:3, 3],hand_matrices[12][:3, 3]))
    result[3,1] = to_unsigned_angle(calc_finger_abduction_angle(hand_matrices[0][:3, 3],hand_matrices[10][:3, 3], hand_matrices[15][:3, 3],hand_matrices[17][:3, 3]))
    result[4,1] = to_unsigned_angle(calc_finger_abduction_angle(hand_matrices[0][:3, 3],hand_matrices[10][:3, 3], hand_matrices[20][:3, 3],hand_matrices[22][:3, 3]))

    return result

# 添加仿真手角度范围配置
@dataclass(frozen=True)
class SimHandCfg:
    min: float | None = None   # 仿真手角度下限（度）
    max: float | None = None   # 仿真手角度上限（度）
    reverse: bool = False      # 是否反向
    fixed: float | None = None # 固定输出值

# 仿真手角度范围配置（按手指 x 关节）
SIM_HAND_CFG: dict[str, tuple[SimHandCfg, SimHandCfg, SimHandCfg, SimHandCfg]] = {
    "thumb": (
    SimHandCfg(min=-0.00808021,  max=1.61378318),                    # F1J1
        SimHandCfg(min=-0.1359066,   max=0.90385146),                # F1J2
        SimHandCfg(min=-0.45214088,  max=1.58660483),                # F1J3
        SimHandCfg(min=-0.45042311,  max=1.58488697),                # F1J4
    ),
    "index": (
        SimHandCfg(min=-0.30216894,  max=1.61116594),                # F2J1
        SimHandCfg(min=-0.36812817,  max=0.36812817),                # F2J2
        SimHandCfg(min=-0.45146008,  max=1.58592416),                # F2J3
        SimHandCfg(min=-0.45465504,  max=1.5891189),                 # F2J4
    ),
    "middle": (
        SimHandCfg(min=-0.29931744,  max=1.60831444),                # F3J1
        SimHandCfg(min=-0.35770517,  max=0.35770517),                # F3J2
        SimHandCfg(min=-0.45111623,  max=1.58558018),                # F3J3
        SimHandCfg(min=-0.46123204,  max=1.59569599),                # F3J4
    ),
    "ring": (
        SimHandCfg(min=-0.3022962,   max=1.61129308),                # F4J1
        SimHandCfg(min=-0.41875733,  max=0.41875733),                # F4J2
        SimHandCfg(min=-0.45141317,  max=1.58587698),                # F4J3
        SimHandCfg(min=-0.45718545,  max=1.59164944),                # F4J4
    ),
    "pinky": (
        SimHandCfg(min=-0.30919836,  max=1.61819525),                # F5J1
        SimHandCfg(min=-0.37253836,  max=0.37253836),                # F5J2
        SimHandCfg(min=-0.46123363,  max=1.59569767),                # F5J3
        SimHandCfg(min=-0.46287711,  max=1.59734101),                # F5J4
    ),
}

# 在文件开头，和其他配置一起
FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

# 四指中值弧度配置
FINGER_CENTER_RAD = {
    "index": 0.35,   # 食指关节1的中值弧度
    "middle": 0,  # 中指关节1的中值弧度
    "ring": 0,    # 无名指关节1的中值弧度
    "pinky": -0.35,   # 小指关节1的中值弧度
}

def apply_finger_coupling_rad(angles_rad):
    """
    应用手指关节耦合关系（弧度版本）：
    当关节2弯曲时，关节1会逐渐向中心线靠拢
    
    Args:
        angles_rad: (5,4) 手指角度矩阵，单位弧度
    
    Returns:
        (5,4) 应用耦合后的角度矩阵（弧度）
    """
    coupled_angles = angles_rad.copy()
    
    # 对除拇指外的四个手指应用耦合
    for finger_idx in range(1, 5):  # index, middle, ring, pinky
        finger_name = FINGERS[finger_idx]
        
        # 从SIM_HAND_CFG配置中获取关节2的范围（已经是弧度）
        joint2_cfg = SIM_HAND_CFG[finger_name][2]
        joint2_min_rad = joint2_cfg.min
        joint2_max_rad = joint2_cfg.max
        joint2_range_rad = joint2_max_rad - joint2_min_rad
        
        # 从SIM_HAND_CFG配置中获取关节1的范围（已经是弧度）
        joint1_cfg = SIM_HAND_CFG[finger_name][1]
        joint1_min_rad = joint1_cfg.min
        joint1_max_rad = joint1_cfg.max
        
        # 计算关节2的弯曲程度（归一化到[0,1]）
        joint2_angle_rad = angles_rad[finger_idx, 2]
        joint2_normalized = (joint2_angle_rad - joint2_min_rad) / joint2_range_rad
        joint2_normalized = np.clip(joint2_normalized, 0.0, 1.0)
        
        # 计算耦合系数：前0.1不耦合，从0.1开始平滑过渡
        if joint2_normalized <= 0.1:
            coupling_factor = 0.0  # 前10%不耦合
        else:
            # 从0.1到1.0平滑过渡，重新映射到[0,1]
            coupling_factor = (joint2_normalized - 0.1) / (1.0 - 0.1)
            coupling_factor = np.clip(coupling_factor, 0.0, 1.0)
        
        # 获取当前关节1的角度和中心值
        current_joint1_rad = angles_rad[finger_idx, 1]  # 原始数据
        joint1_center_rad = FINGER_CENTER_RAD[finger_name]  # 中值
        
        # 平滑加权混合：随着joint2增大，原始数据权重减小，中值权重增大
        original_weight = 1.0 - coupling_factor * 0.4
        center_weight = coupling_factor * 0.8
        
        # 加权混合
        coupled_angles[finger_idx, 1] = (current_joint1_rad * original_weight + 
                                        joint1_center_rad * center_weight)
        
        # 限制在SIM_HAND_CFG的范围内（弧度）
        coupled_angles[finger_idx, 1] = np.clip(coupled_angles[finger_idx, 1], joint1_min_rad, joint1_max_rad)
    
    return coupled_angles

def map_visionpro_to_simhand_rad(control_angles_deg, visionpro_cfg=None, simhand_cfg=None):
    """
    将Vision Pro角度映射到仿真手弧度
    
    Args:
        control_angles_deg: (5,4) Vision Pro计算出的角度，单位度
        visionpro_cfg: Vision Pro角度配置，如果为None则使用CFG
        simhand_cfg: 仿真手角度配置，如果为None则使用SIM_HAND_CFG
    
    Returns:
        (5,4) 仿真手弧度值
    """
    if visionpro_cfg is None:
        visionpro_cfg = CFG
    if simhand_cfg is None:
        simhand_cfg = SIM_HAND_CFG
    
    simhand_angles_rad = np.zeros_like(control_angles_deg)
    
    for i, finger in enumerate(FINGERS):
        for j in range(4):
            vp_cfg = visionpro_cfg[finger][j]
            sh_cfg = simhand_cfg[finger][j]
            
            # 如果仿真手配置为固定值
            if sh_cfg.fixed is not None:
                simhand_angles_rad[i, j] = sh_cfg.fixed
                continue
            
            # 如果Vision Pro配置为固定值，映射到仿真手范围的中点
            if vp_cfg.fixed is not None:
                simhand_angles_rad[i, j] = np.radians((sh_cfg.min + sh_cfg.max) / 2.0)
                continue
            
            # 获取Vision Pro角度值
            vp_angle = control_angles_deg[i, j]
            
            # 将Vision Pro角度限制在其配置范围内
            vp_min = vp_cfg.min
            vp_max = vp_cfg.max
            vp_clamped = np.clip(vp_angle, vp_min, vp_max)
            
            # 计算在Vision Pro范围内的归一化位置 [0,1]
            if abs(vp_max - vp_min) < 1e-6:
                normalized = 0.5
            else:
                if vp_cfg.reverse:
                    normalized = (vp_max - vp_clamped) / (vp_max - vp_min)
                else:
                    normalized = (vp_clamped - vp_min) / (vp_max - vp_min)
            
            # 映射到仿真手范围并转换为弧度
            sh_min = sh_cfg.min
            sh_max = sh_cfg.max
            if sh_cfg.reverse:
                simhand_angle_deg = sh_max - normalized * (sh_max - sh_min)
            else:
                simhand_angle_deg = sh_min + normalized * (sh_max - sh_min)
            
            simhand_angles_rad[i, j] = simhand_angle_deg

    result = apply_finger_coupling_rad(simhand_angles_rad)
    
    return result

# 在文件开头添加全局变量
_hand_matrices_buffer = None
_joint_angles_buffer = None
_buffer_size = 5  # 使用5帧数据进行平滑
_smoothing_factor = 0.4

def smooth_hand_matrices(hand_matrices):
    """对VisionPro手部矩阵数据进行多帧平滑处理"""
    global _hand_matrices_buffer
    
    if _hand_matrices_buffer is None:
        # 初始化缓冲区
        _hand_matrices_buffer = [hand_matrices.copy() for _ in range(_buffer_size)]
        return hand_matrices
    
    # 添加新帧到缓冲区
    _hand_matrices_buffer.append(hand_matrices.copy())
    
    # 保持缓冲区大小
    if len(_hand_matrices_buffer) > _buffer_size:
        _hand_matrices_buffer.pop(0)
    
    # 计算加权平均（最近的帧权重更大）
    weights = np.linspace(0.1, 1.0, len(_hand_matrices_buffer))  # 权重递增
    weights = weights / np.sum(weights)  # 归一化权重
    
    # 加权平均
    smoothed_matrices = np.zeros_like(hand_matrices)
    for i, (frame, weight) in enumerate(zip(_hand_matrices_buffer, weights)):
        smoothed_matrices += weight * frame
    
    return smoothed_matrices

def smooth_joint_angles(joint_angles):
    """对关节角度进行多帧平滑处理"""
    global _joint_angles_buffer
    
    if _joint_angles_buffer is None:
        # 初始化缓冲区
        _joint_angles_buffer = [joint_angles.copy() for _ in range(_buffer_size)]
        return joint_angles
    
    # 添加新帧到缓冲区
    _joint_angles_buffer.append(joint_angles.copy())
    
    # 保持缓冲区大小
    if len(_joint_angles_buffer) > _buffer_size:
        _joint_angles_buffer.pop(0)
    
    # 计算加权平均（最近的帧权重更大）
    weights = np.linspace(0.1, 1.0, len(_joint_angles_buffer))  # 权重递增
    weights = weights / np.sum(weights)  # 归一化权重
    
    # 加权平均
    smoothed_angles = np.zeros_like(joint_angles)
    for i, (frame, weight) in enumerate(zip(_joint_angles_buffer, weights)):
        smoothed_angles += weight * frame
    
    return smoothed_angles

def retarget_wuji_hand(hand_matrices, filter=None):
    """
    hand_matrices: 25×4×4
    return: 20 通道弧度值（flatten）
    """
    # 先对原始数据进行平滑处理
    smoothed_matrices = smooth_hand_matrices(hand_matrices)
    
    control_angles = solve_hand_angles(smoothed_matrices)  # (5,4) 度

    if filter is not None:
        control_angles = filter.step(control_angles)

    # 对最终关节角度进行二次平滑
    control_angles = smooth_joint_angles(control_angles)

    quantum_deg = 0.1
    control_angles = np.round(control_angles / quantum_deg) * quantum_deg

    # 标定阶段需要就打开：持续累积各通道 min/max
    # **************************************************************
    record_angle_stats(control_angles)
    # **************************************************************

    print(f"control_angles: {control_angles}")

    simhand_rad = map_visionpro_to_simhand_rad(control_angles)

    simhand_rad = np.round(simhand_rad, 3)

    # print(f"simhand_rad: {simhand_rad}")
    
    return simhand_rad
    # return simhand_rad
