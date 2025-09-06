import time
from avp_stream import VisionProStreamer
from mediapipe_utils import convert_vision_pro_to_mediapipe_format, RetargetingConfig
from wuji_hand_pdo import WujiHand
from visionpro_map_utils import retarget_wuji_hand
#from wuji_hand_sdo import WujiHand


class MPVisionProController:
    def __init__(
        self,
        tpdo_id_right: int = 1,
        tpdo_id_left: int = 1,
        pdo_interval: int = 1000,
        side: str = "right",  # "right" | "left" | "both"
        config_path_right: str = None,
        config_path_left: str = None,
    ):
        self.side = side.lower()
        assert self.side in {"right", "left", "both"}, "side must be 'right', 'left', or 'both'"

        # VisionPro stream
        self.s = VisionProStreamer(ip="192.168.50.191", record=False)

        # Hands
        self.right_hand = None
        self.left_hand = None
        try:
            if self.side in {"right", "both"}:
                self.right_hand = WujiHand(tpdo_id=tpdo_id_right, interval=pdo_interval)
                print("Wuji RIGHT hand PDO controller initialized successfully")
            if self.side in {"left", "both"}:
                self.left_hand = WujiHand(tpdo_id=tpdo_id_left, interval=pdo_interval)
                #self.left_hand = WujiHand()
                print("Wuji LEFT hand PDO controller initialized successfully")
        except Exception as e:
            print(f"Failed to initialize WujiHand: {e}")
            raise

        self.init_dex_retargeter(config_path_right, config_path_left)

    def init_dex_retargeter(self, config_path_r=None, config_path_l=None):
        from dex_retargeting.retargeting_config import RetargetingConfig

        if self.side in {"right", "both"} and config_path_r:
            config_r = RetargetingConfig.load_from_file(config_path_r)
            self.retarget_right = config_r.build()
            self.retargeting_type_right = self.retarget_right.optimizer.retargeting_type
            self.indices_right = self.retarget_right.optimizer.target_link_human_indices

        if self.side in {"left", "both"} and config_path_l:
            config_l = RetargetingConfig.load_from_file(config_path_l)
            self.retarget_left = config_l.build()
            self.retargeting_type_left = self.retarget_left.optimizer.retargeting_type
            self.indices_left = self.retarget_left.optimizer.target_link_human_indices

    def __del__(self):
        try:
            if getattr(self, "right_hand", None) is not None:
                self.right_hand.cleanup()
        except Exception:
            pass
        try:
            if getattr(self, "left_hand", None) is not None:
                self.left_hand.cleanup()
        except Exception:
            pass

    def process_right_hand_data(self):
        """Process VisionPro RIGHT-hand data and send to robot."""
        if self.right_hand is None:
            return
        try:
            r = self.s.latest
            right_fingers_mat = r["right_fingers"]  # (25, 4, 4)

            human_mediapipe_pose = convert_vision_pro_to_mediapipe_format(
                right_fingers_mat, hand_type="Right"
            )

            if self.retargeting_type_right == "POSITION":
                ref_value = human_mediapipe_pose[self.indices_right, :]
            else:  # VECTOR
                origin_indices = self.indices_right[0, :]
                task_indices = self.indices_right[1, :]
                ref_value = human_mediapipe_pose[task_indices, :] - human_mediapipe_pose[origin_indices, :]

            robot_qpos = self.retarget_right.retarget(ref_value)
            hand_positions = robot_qpos.reshape(5, 4)
            self.right_hand.set_joint_positions_pdo(hand_positions)

        except Exception as e:
            print(f"[RIGHT] Error: {e}")

    def process_left_hand_data(self):
        """Process VisionPro LEFT-hand data and send to robot."""
        if self.left_hand is None:
            return
        try:
            r = self.s.latest
            left_fingers_mat = r["left_fingers"]  # (25, 4, 4) — adjust key if your stream differs

            joint_commands = retarget_wuji_hand(left_fingers_mat)  # 不传self.rh_filter


            human_mediapipe_pose = convert_vision_pro_to_mediapipe_format(
                left_fingers_mat, hand_type="Left"
            )

            if self.retargeting_type_left == "POSITION":
                ref_value = human_mediapipe_pose[self.indices_left, :]
            else:  # VECTOR
                origin_indices = self.indices_left[0, :]
                task_indices = self.indices_left[1, :]
                ref_value = human_mediapipe_pose[task_indices, :] - human_mediapipe_pose[origin_indices, :]

            robot_qpos = self.retarget_left.retarget(ref_value)
            print('robot_qpos: ', robot_qpos)
            print('joint_commands: ', joint_commands)

            hand_positions = robot_qpos.reshape(5, 4)
            #hand_positions = 1 * hand_positions + 0 * joint_commands

            hand_positions[0, 2] = joint_commands[0, 2]
            hand_positions[0, 3] = joint_commands[0, 3]



            hand_positions[1, 0] = joint_commands[1, 0]
            hand_positions[1, 2] = joint_commands[1, 2]

            hand_positions[2, 0] = joint_commands[2, 0]
            hand_positions[2, 2] = joint_commands[2, 2]

            hand_positions[3, 0] = joint_commands[3, 0]
            hand_positions[3, 2] = joint_commands[3, 2]

            hand_positions[4, 0] = joint_commands[4, 0]
            hand_positions[4, 2] = joint_commands[4, 2]


            hand_positions[1, 1] = -1 * hand_positions[1, 1]
            hand_positions[2, 1] = -1 * hand_positions[2, 1]
            hand_positions[3, 1] = -1 * hand_positions[3, 1]
            hand_positions[4, 1] = -1 * hand_positions[4, 1]
            #hand_positions[4, 3] = 0.2 * hand_positions[4, 3]

            print('input_data: ', hand_positions)
            self.left_hand.set_joint_positions_pdo(hand_positions)
            #self.left_hand.set_joint_positions_sdo(hand_positions)

        except Exception as e:
            print(f"[LEFT] Error: {e}")

    def process_once(self):
        if self.side == "right":
            self.process_right_hand_data()
        elif self.side == "left":
            self.process_left_hand_data()
        else:  # both
            self.process_right_hand_data()
            self.process_left_hand_data()

    def run(self):
        """Run controller loop."""
        print(f"Starting VisionPro Hand Controller (side={self.side})...")
        print("Press Ctrl+C to stop")
        try:
            while True:
                self.process_once()
                time.sleep(0.01)  # ~100 Hz
        except KeyboardInterrupt:
            print("Stopping controller...")
        finally:
            if self.right_hand is not None:
                self.right_hand.cleanup()
            if self.left_hand is not None:
                self.left_hand.cleanup()


if __name__ == "__main__":
    controller = MPVisionProController(
        side="left",
        config_path_left=RetargetingConfig.DEX_LEFT.value,
    )
    controller.run()
