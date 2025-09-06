import time
import pickle
import threading
import os
from datetime import datetime
from ..visionpro_hand_controller import VisionProController
from ..visionpro_map_utils import retarget_wuji_hand

class VisionProRecorder:
    def __init__(self, tpdo_id: int = 1, pdo_interval: int = 1000):
        # 导入控制器
        self.controller = VisionProController(tpdo_id, pdo_interval)
        
        # 记录功能
        self.recording = False
        self.recorded_data = []
        self.recording_thread = None
        
        # 创建记录目录
        self.record_dir = os.path.join(os.path.dirname(__file__), 'recorded_actions')
        os.makedirs(self.record_dir, exist_ok=True)

    def toggle_recording(self):
        """切换记录状态"""
        if not self.recording:
            self.start_recording()
        else:
            self.stop_recording()

    def start_recording(self):
        """开始记录"""
        self.recording = True
        self.recorded_data = []
        print("🔴 开始记录动作...")
        
        # 启动记录线程
        self.recording_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.recording_thread.start()

    def stop_recording(self):
        """停止记录"""
        self.recording = False
        if self.recording_thread:
            self.recording_thread.join()
        
        # 保存数据到recorded_actions目录
        if self.recorded_data:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"recorded_action_{timestamp}.pkl"
            filepath = os.path.join(self.record_dir, filename)
            
            with open(filepath, 'wb') as f:
                pickle.dump(self.recorded_data, f)
            
            print(f"✅ 记录完成，保存到: {filepath}")
            print(f"   共记录 {len(self.recorded_data)} 帧数据")
        else:
            print("❌ 没有记录到数据")

    def _record_loop(self):
        """记录循环（在单独线程中运行）"""
        while self.recording:
            try:
                r = self.controller.s.latest
                right_fingers_mat = r["right_fingers"]
                
                # 直接使用import的函数
                joint_commands = retarget_wuji_hand(right_fingers_mat)
                
                # 记录关节数据（5x4格式）
                hand_positions = joint_commands.reshape(5, 4)
                
                self.recorded_data.append({
                    'timestamp': time.time(),
                    'hand_positions': hand_positions.copy()
                })
                
                time.sleep(0.01)  # 100Hz
            except Exception as e:
                print(f"记录错误: {e}")

    def run(self):
        """运行记录器"""
        print("Starting VisionPro Recorder...")
        print("按空格键开始/停止记录动作")
        print("Press Ctrl+C to stop")
        
        # 添加键盘监听
        import keyboard
        keyboard.on_press_key('space', lambda _: self.toggle_recording())
        
        try:
            while True:
                # 运行控制器（实时控制）  
                self.controller.process_visionpro_data()
                time.sleep(0.01)  # 100Hz
        except KeyboardInterrupt:
            print("Stopping recorder...")

if __name__ == '__main__':
    recorder = VisionProRecorder()
    recorder.run()