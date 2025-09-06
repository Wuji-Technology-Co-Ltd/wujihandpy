import wujihandpy
import numpy as np
import time
import math

class WujiHand:
    def __init__(self, tpdo_id: int = 1, interval: int = 1000):
        """
        初始化WujiHand PDO控制器
        Args:
            tpdo_id: PDO传输ID，默认为1
            interval: PDO更新间隔(微秒)，默认为1000微秒(1ms)
        """
        try:
            self.hand = wujihandpy.Hand(usb_pid=0x2000)
            print("设备连接成功!")
        except Exception as e:
            print(f"设备连接失败: {e}")
            raise
        self.tpdo_id = tpdo_id
        self.interval = interval
        self._setup_pdo_mode()

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

    def _setup_pdo_mode(self):
        """设置PDO模式"""
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
            self.hand.write_joint_control_word(np.uint16(5))

            self.hand.write_joint_control_mode(np.uint16(4))  # 设置为CSP模式
            self.hand.write_global_tpdo_id(np.uint16(self.tpdo_id))  # 设置PDO ID
            self.hand.write_pdo_interval(np.uint32(self.interval))  # 设置更新间隔
            self.hand.write_pdo_enabled(np.uint8(1))  # 启用PDO通信
            self.hand.write_joint_control_word(
                np.array(
                    [
                        # J1J2 J3J4
                        [1, 1, 1, 1],  # F1
                        [1, 1, 1, 1],  # F2
                        [1, 1, 1, 1],  # F3
                        [1, 1, 1, 1],  # F4
                        [1, 1, 1, 1],  # F5
                    ],
                    dtype=np.uint16,
                )
            )

            print("PDO模式设置完成")
        except Exception as e:
            print(f"设置PDO模式失败: {e}")
            raise

    def set_joint_positions_pdo(self, positions: np.ndarray):
        """
        设置关节位置 (PDO模式)
        
        Args:
            positions: 5x4的numpy数组，表示5个手指的4个关节位置
                      或者单个浮点值，应用到所有启用的关节
        """
        try:
            self.hand.pdo_write_unchecked(positions)
        except Exception as e:
            print(f"设置关节位置失败: {e}")

    def set_joint_positions_pdo_single(self, position: float):
        """
        设置所有启用的关节到相同位置 (PDO模式)
        
        Args:
            position: 单个浮点值，应用到所有启用的关节
        """
        try:
            self.hand.pdo_write_unchecked(np.float64(position))
        except Exception as e:
            print(f"设置关节位置失败: {e}")

    def run_sine_wave_control(self, frequency: float = 1.0, amplitude: float = 0.8, duration: float = 10.0):
        """
        运行正弦波控制示例
        
        Args:
            frequency: 正弦波频率 (Hz)
            amplitude: 正弦波幅度
            duration: 运行时长 (秒)
        """
        update_rate = 1000.0  # 1kHz
        update_period = 1.0 / update_rate
        start_time = time.time()
        x = 0
        print(f"开始正弦波控制: 频率={frequency}Hz, 幅度={amplitude}, 时长={duration}秒")
        while time.time() - start_time < duration:
            y = (1 - math.cos(x)) * amplitude
            self.set_joint_positions_pdo_single(y)
            x += math.pi * frequency / update_rate
            time.sleep(update_period)
        print("正弦波控制完成")

if __name__ == "__main__":
    try:
        # 创建WujiHand实例
        hand = WujiHand(tpdo_id=1, interval=1000)
        print("\n示例1: 基本位置控制")
        test_positions = np.array(
            [
                # J1    J2    J3    J4
                [0.0, 0.0, 0.0, 0.0],  # F1
                [0.5, 0.0, 0.5, 0.5],  # F2
                [0.5, 0.0, 0.5, 0.5],  # F3
                [0.5, 0.0, 0.5, 0.5],  # F4
                [0.5, 0.0, 0.5, 0.5],  # F5
            ],
            dtype=np.float64,
        )
        hand.set_joint_positions_pdo(test_positions)
        time.sleep(2)
        
        # 示例2: 单个值控制所有关节
        print("\n示例2: 单个值控制")
        hand.set_joint_positions_pdo_single(0.3)
        time.sleep(2)
        
        # 示例3: 正弦波控制
        print("\n示例3: 正弦波控制")
        hand.run_sine_wave_control(frequency=0.5, amplitude=0.6, duration=5.0)
        
        # 示例4: 高频控制循环
        print("\n示例4: 高频控制循环")
        start_time = time.time()
        for i in range(100):  # 运行100次 (100ms)
            # 计算正弦波位置
            t = time.time() - start_time
            y = 0.4 * math.sin(2 * math.pi * 2 * t)  # 2Hz正弦波
            
            # 设置位置
            hand.set_joint_positions_pdo_single(y)
            
            # 精确等待1ms
            time.sleep(0.001)
        
        print("所有示例运行完成!")
        
    except Exception as e:
        print(f"示例运行出错: {e}")
    finally:
        # 清理资源
        if 'hand' in locals():
            hand.cleanup()