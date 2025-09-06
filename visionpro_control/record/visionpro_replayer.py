import time
import pickle
import os
import sys
current_dir = os.path.dirname(__file__)
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from wuji_hand_pdo import WujiHand

class VisionProReplayer:
    def __init__(self, tpdo_id: int = 1, pdo_interval: int = 1000):
        # 初始化机械手
        try:
            self.hand = WujiHand(tpdo_id=tpdo_id, interval=pdo_interval)
            print('WujiHand PDO controller initialized successfully')
        except Exception as e:
            print(f'Failed to initialize WujiHand: {e}')
            raise
        
        # 记录目录
        self.record_dir = os.path.join(os.path.dirname(__file__), 'recorded_actions')

    def list_recorded_files(self):
        """列出所有记录文件"""
        if not os.path.exists(self.record_dir):
            return []
        
        files = [f for f in os.listdir(self.record_dir) if f.startswith('recorded_action_') and f.endswith('.pkl')]
        files.sort(reverse=True)  # 按时间倒序
        return files

    def load_recording(self, filename):
        """加载记录文件"""
        filepath = os.path.join(self.record_dir, filename)
        try:
            with open(filepath, 'rb') as f:
                data = pickle.load(f)
            print(f"✅ 加载记录文件: {filepath}")
            print(f"   共 {len(data)} 帧数据")
            return data
        except Exception as e:
            print(f"❌ 加载文件失败: {e}")
            return None

    def replay(self, data, speed_multiplier=1.0, loop=False):
        """
        回放记录的动作
        
        Args:
            data: 记录的数据列表
            speed_multiplier: 速度倍数，1.0=正常速度，2.0=2倍速，0.5=0.5倍速
            loop: 是否循环播放
        """
        if not data:
            print("❌ 没有数据可回放")
            return
        
        print(f"🎬 开始回放动作...")
        print(f"   速度倍数: {speed_multiplier}x")
        print(f"   循环播放: {'是' if loop else '否'}")
        print("   按 Ctrl+C 停止回放")
        
        try:
            while True:
                for i, frame in enumerate(data):
                    hand_positions = frame['hand_positions']
                    
                    # 发送到机械手
                    self.hand.set_joint_positions_pdo(hand_positions)
                    
                    # 计算延迟时间
                    if i < len(data) - 1:
                        # 使用实际时间间隔
                        time_diff = data[i+1]['timestamp'] - frame['timestamp']
                        delay = time_diff / speed_multiplier
                        time.sleep(max(0.001, delay))  # 最小1ms延迟
                    else:
                        # 最后一帧，使用默认延迟
                        time.sleep(0.01 / speed_multiplier)
                
                if not loop:
                    break
                else:
                    print("🔄 循环播放...")
                    
        except KeyboardInterrupt:
            print("⏹️ 回放已停止")
        except Exception as e:
            print(f"❌ 回放错误: {e}")

    def interactive_replay(self):
        """交互式回放"""
        # 列出文件
        files = self.list_recorded_files()
        if not files:
            print("❌ 没有找到记录文件")
            return
        
        print(" 可用的记录文件:")
        for i, filename in enumerate(files):
            print(f"   {i+1}. {filename}")
        
        # 选择文件
        try:
            choice = int(input("\n请选择文件编号: ")) - 1
            if 0 <= choice < len(files):
                filename = files[choice]
            else:
                print("❌ 无效选择")
                return
        except ValueError:
            print("❌ 请输入数字")
            return
        
        # 加载数据
        data = self.load_recording(filename)
        if not data:
            return
        
        # 设置回放参数
        try:
            speed = float(input("请输入速度倍数 (默认1.0): ") or "1.0")
            loop_input = input("是否循环播放? (y/N): ").lower()
            loop = loop_input in ['y', 'yes', '是']
        except ValueError:
            speed = 1.0
            loop = False
        
        # 开始回放
        self.replay(data, speed, loop)

    def run(self):
        """运行回放器"""
        print("Starting VisionPro Replayer...")
        
        try:
            self.interactive_replay()
        except KeyboardInterrupt:
            print("Stopping replayer...")
        finally:
            self.hand.cleanup()

if __name__ == '__main__':
    replayer = VisionProReplayer()
    replayer.run() 