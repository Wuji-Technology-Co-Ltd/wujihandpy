import wujihandpy
import numpy as np
import time
import math


def main():
    hand = wujihandpy.Hand()
    try:
        run(hand)
    finally:
        # Disable the entire hand
        hand.write_joint_enabled(False)


def run(hand: wujihandpy.Hand):
    # Enable all joints
    hand.write_joint_enabled(True)

    with hand.realtime_controller(
        enable_upstream=False, filter=wujihandpy.filter.LowPass(cutoff_freq=2.0)
    ) as controller:
        # Filtered realtime control (100Hz -> 1kHz)
        update_rate = 100.0
        update_period = 1.0 / update_rate

        x = 0
        while True:
            y = (1 - math.cos(x)) * 0.8
            controller.set_joint_target_position(
                np.array(
                    [
                        # J1J2 J3J4
                        [0, 0, 0, 0],  # F1
                        [y, 0, y, y],  # F2
                        [y, 0, y, y],  # F3
                        [y, 0, y, y],  # F4
                        [y, 0, y, y],  # F5
                    ],
                    dtype=np.float64,
                )
            )

            x += math.pi / update_rate
            time.sleep(update_period)


if __name__ == "__main__":
    main()
