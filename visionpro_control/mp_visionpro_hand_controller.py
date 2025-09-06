import time
from avp_stream import VisionProStreamer
from visionpro_map_utils import retarget_wuji_hand
from mediapipe_utils import convert_vision_pro_to_mediapipe_format
from wuji_hand_pdo import WujiHand
# from wuji_hand_sdo import WujiHand

class MPVisionProController:
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

        self.init_dex_retargeter()

    def init_dex_retargeter(self):
        from dex_retargeting.retargeting_config import RetargetingConfig
        config_path = "/home/uji/Robben_ws/wujihandpy/visionpro_control/wujihand_right_dexpilot.yaml"
        config = RetargetingConfig.load_from_file(config_path)
        self.retarget = config.build()
        # Convert landmarks to vectors for vector retargeting
        self.retargeting_type = self.retarget.optimizer.retargeting_type
        self.indices = self.retarget.optimizer.target_link_human_indices


    def __del__(self):
        """析构函数，自动清理资源"""
        if hasattr(self, 'hand') and self.hand is not None:
            self.hand.cleanup()

    def process_visionpro_data(self):
        """处理 VisionPro 数据并发送到机械手"""
        try:
            r = self.s.latest
            right_fingers_mat = r["right_fingers"]  # (25, 4, 4)
            
            human_mediapipe_pose = convert_vision_pro_to_mediapipe_format(right_fingers_mat, hand_type="Right")
                
            if self.retargeting_type == "POSITION":
                ref_value = human_mediapipe_pose[self.indices, :]
            else:  # VECTOR retargeting
                origin_indices = self.indices[0, :]
                task_indices = self.indices[1, :]
                ref_value = human_mediapipe_pose[task_indices, :] - human_mediapipe_pose[origin_indices, :]
                
            robot_qpos = self.retarget.retarget(ref_value)

            # 转换为5x4格式并发送
            hand_positions = robot_qpos.reshape(5, 4)
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
