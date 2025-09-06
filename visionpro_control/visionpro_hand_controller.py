import time
from avp_stream import VisionProStreamer
from visionpro_map_utils import retarget_wuji_hand
from wuji_hand_pdo import WujiHand
# from wuji_hand_sdo import WujiHand

class VisionProController:
    def __init__(self, tpdo_id: int = 1, pdo_interval: int = 1000):
        # VisionPro stream
        self.s = VisionProStreamer(ip='192.168.50.191', record=False)
        
        # 注释掉重复的滤波，utils里已经有多帧平滑了
        # from visionpro_map_utils import LowPassEMA
        # self.rh_filter = LowPassEMA(alpha=0.5)
        
        # 初始化机械手
        try:
            self.hand = WujiHand(tpdo_id=tpdo_id, interval=pdo_interval)
            print('WujiHand PDO controller initialized successfully')
        except Exception as e:
            print(f'Failed to initialize WujiHand: {e}')
            raise

    def __del__(self):
        """析构函数，自动清理资源"""
        if hasattr(self, 'hand') and self.hand is not None:
            self.hand.cleanup()

    def process_visionpro_data(self):
        """处理 VisionPro 数据并发送到机械手"""
        try:
            r = self.s.latest
            # right_fingers_mat = r["right_fingers"]  # (25, 4, 4)
            left_fingers_mat = r["left_fingers"]  # (25, 4, 4)

            # 只使用utils里的多帧平滑，不传filter参数
            joint_commands = retarget_wuji_hand(left_fingers_mat)  # 不传self.rh_filter
            
            # 转换为5x4格式并发送
            hand_positions = joint_commands.reshape(5, 4)
            hand_positions[1, 1] = -1 * hand_positions[1, 1]
            hand_positions[2, 1] = -1 * hand_positions[2, 1]
            hand_positions[3, 1] = -1 * hand_positions[3, 1]
            hand_positions[4, 1] = -1 * hand_positions[4, 1]            
            self.hand.set_joint_positions_pdo(hand_positions)
            
        except Exception as e:
            print(f'Error: {e}')

    def run(self):
        """运行控制器"""
        print("Starting VisionPro Hand Controller...")
        print("Press Ctrl+C to stop")
        
        try:
            while True:
                self.process_visionpro_data()
                time.sleep(0.01)  # 100Hz
        except KeyboardInterrupt:
            print("Stopping controller...")
        finally:
            self.hand.cleanup()

if __name__ == '__main__':
    controller = VisionProController()
    controller.run()
