import wujihandpy
import numpy as np
import time
import math

class WujiHand:
    def __init__(self):
        """
        初始化WujiHand SDO控制器
        """
        try:
            self.hand = wujihandpy.Hand(usb_pid=0x2000)
            print("设备连接成功!")
        except Exception as e:
            print(f"设备连接失败: {e}")
            raise
        self._setup_sdo_mode()
        
        # 记录当前位置用于插值
        self.current_positions = np.array(
            [
                # J1    J2    J3    J4
                [+0.0, +0.0, +0.0, +0.0],  # F1 (拇指)
                [+0.0, +0.1, +0.0, +0.0],  # F2 (食指)
                [+0.0, +0.0, +0.0, +0.0],  # F3 (中指)
                [+0.0, +0.0, +0.0, +0.0],  # F4 (无名指)
                [+0.0, -0.1, +0.0, +0.0],  # F5 (小指)
            ],
            dtype=np.float64,
        )

    def __del__(self):
        """析构函数，自动清理资源"""
        try:
            if hasattr(self, 'hand') and self.hand:
                self.hand.write_joint_control_word(np.uint16(5))
                print("手部已禁用")
        except Exception as e:
            print(f"清理资源时出错: {e}")

    def cleanup(self):
        """手动清理资源"""
        try:
            if hasattr(self, 'hand') and self.hand:
                self.hand.write_joint_control_word(np.uint16(5))
                print("手部已禁用")
        except Exception as e:
            print(f"清理资源时出错: {e}")

    def _setup_sdo_mode(self):
        """设置SDO模式"""
        try:
            self.hand.write_joint_control_mode(np.uint16(2))
            self.hand.write_joint_control_word(np.uint16(1))
            self.hand.write_joint_control_position(
                np.array(
                    [
                        # J1    J2    J3    J4
                        [+0.0, +0.0, +0.0, +0.0],  # F1 (拇指)
                        [+0.0, +0.1, +0.0, +0.0],  # F2 (食指)
                        [+0.0, +0.0, +0.0, +0.0],  # F3 (中指)
                        [+0.0, +0.0, +0.0, +0.0],  # F4 (无名指)
                        [+0.0, -0.1, +0.0, +0.0],  # F5 (小指)
                    ],
                    dtype=np.float64,
                )
            )
            time.sleep(0.5)
            self.hand.write_joint_control_word(
                np.array(
                    [
                        # J1J2 J3J4
                        [1, 1, 1, 1],  # F1
                        [1, 5, 1, 1],  # F2
                        [1, 5, 1, 1],  # F3
                        [1, 5, 1, 1],  # F4
                        [1, 5, 1, 1],  # F5
                    ],
                    dtype=np.uint16,
                )
            )

            print("SDO模式设置完成")
        except Exception as e:
            print(f"设置SDO模式失败: {e}")
            raise

    def set_joint_positions_sdo(self, positions: np.ndarray, interpolation_steps: int = 10, step_duration: float = 0.001):
        """
        设置关节位置 (SDO模式) - 带插值
        
        Args:
            positions: 5x4的numpy数组，表示5个手指的4个关节位置
            interpolation_steps: 插值步数，默认20步
            step_duration: 每步持续时间(秒)，默认0.05秒
        """
        try:
            # 确保输入是5x4数组
            if positions.shape != (5, 4):
                if positions.size == 1:
                    # 如果是单个值，扩展到5x4
                    positions = np.full((5, 4), float(positions), dtype=np.float64)
                else:
                    raise ValueError(f"Expected shape (5,4) or single value, got {positions.shape}")
            
            # 计算插值路径
            start_positions = self.current_positions.copy()
            end_positions = positions.astype(np.float64)
            
            # 线性插值
            for step in range(interpolation_steps + 1):
                # 计算插值系数 (0 到 1)
                t = step / interpolation_steps
                
                # 线性插值
                interpolated_positions = start_positions + t * (end_positions - start_positions)
                
                # 发送到机械手
                self.hand.write_joint_control_position(interpolated_positions)
                
                # 等待
                if step < interpolation_steps:
                    time.sleep(step_duration)
            
            # 更新当前位置
            self.current_positions = end_positions.copy()
            
        except Exception as e:
            print(f"设置关节位置失败: {e}")

    def set_joint_positions_sdo_single(self, position: float, interpolation_steps: int = 20, step_duration: float = 0.05):
        """
        设置所有启用的关节到相同位置 (SDO模式) - 带插值
        
        Args:
            position: 单个浮点值，应用到所有启用的关节
            interpolation_steps: 插值步数，默认20步
            step_duration: 每步持续时间(秒)，默认0.05秒
        """
        try:
            # 创建5x4数组，所有位置设为相同值
            positions = np.full((5, 4), float(position), dtype=np.float64)
            self.set_joint_positions_sdo(positions, interpolation_steps, step_duration)
        except Exception as e:
            print(f"设置关节位置失败: {e}")

    def run_sine_wave_control(self, frequency: float = 1.0, amplitude: float = 0.8, duration: float = 10.0):
        """
        运行正弦波控制示例 (SDO模式)
        
        Args:
            frequency: 正弦波频率 (Hz)
            amplitude: 正弦波幅度
            duration: 运行时长 (秒)
        """
        update_rate = 10.0  # 10Hz for SDO mode
        update_period = 1.0 / update_rate
        start_time = time.time()
        x = 0
        print(f"开始正弦波控制: 频率={frequency}Hz, 幅度={amplitude}, 时长={duration}秒")
        while time.time() - start_time < duration:
            y = (1 - math.cos(x)) * amplitude
            # 使用插值，但步数较少以保持响应性
            self.set_joint_positions_sdo_single(y, interpolation_steps=5, step_duration=0.02)
            x += math.pi * frequency / update_rate
            time.sleep(update_period)
        print("正弦波控制完成")

if __name__ == "__main__":
    try:
        # 创建WujiHand实例
        hand = WujiHand()
        
        print("\n示例1: 基本位置控制 (带插值)")
        test_positions = np.array(
            [
                # J1    J2    J3    J4
                [0.0, 0.0, 0.0, 0.0],  # F1
                [0.5, 0.0, 0.5, 0.5],  # F2
                [0.5, 0.0, 0.5, 0.5],  # F3
                [0.5, 0.0, 0.5, 0.5],  # F4
                [0.0, 0.0, 0.0, 0.0],  # F5 (小指禁用，位置无效)
            ],
            dtype=np.float64,
        )
        hand.set_joint_positions_sdo(test_positions, interpolation_steps=20, step_duration=0.05)
        
        # 示例2: 单个值控制所有启用的关节 (带插值)
        print("\n示例2: 单个值控制 (带插值)")
        hand.set_joint_positions_sdo_single(0.3, interpolation_steps=15, step_duration=0.03)
        
        # 示例3: 快速插值测试
        print("\n示例3: 快速插值测试")
        hand.set_joint_positions_sdo_single(0.8, interpolation_steps=10, step_duration=0.02)
        time.sleep(1)
        hand.set_joint_positions_sdo_single(0.2, interpolation_steps=10, step_duration=0.02)
        
        # 示例4: 逐步控制 (带插值)
        print("\n示例4: 逐步控制 (带插值)")
        for i in range(5):
            pos = 0.1 * (i + 1)
            print(f"设置位置: {pos}")
            hand.set_joint_positions_sdo_single(pos, interpolation_steps=8, step_duration=0.04)
        
        print("所有示例运行完成!")
        
    except Exception as e:
        print(f"示例运行出错: {e}")
    finally:
        # 清理资源
        if 'hand' in locals():
            hand.cleanup()