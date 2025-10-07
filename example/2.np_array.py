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

    # Return all joints to initial point
    hand.write_joint_target_position(
        np.array(
            [
                # J1    J2    J3    J4
                [+0.0, +0.0, +0.0, +0.0],  # F1
                [+0.0, +0.1, +0.0, +0.0],  # F2
                [+0.0, +0.0, +0.0, +0.0],  # F3
                [+0.0, +0.0, +0.0, +0.0],  # F4
                [+0.0, -0.1, +0.0, +0.0],  # F5
            ],
            dtype=np.float64,
        )
    )

    # Wait for joints to move into place
    time.sleep(0.5)

    # Disable non-middle fingers
    hand.write_joint_enabled(
        np.array(
            [
                # J1J2 J3J4
                [0, 0, 0, 0],  # F1
                [0, 0, 0, 0],  # F2
                [1, 1, 1, 1],  # F3
                [0, 0, 0, 0],  # F4
                [0, 0, 0, 0],  # F5
            ],
            dtype=np.bool,
        )
    )

    # 2Hz SDO Control
    update_rate = 2.0
    update_period = 1.0 / update_rate

    x = 0.0
    while True:
        y = (1 - math.cos(x)) * 0.8

        # Control middle finger
        hand.finger(2).write_joint_target_position_unchecked(
            np.array(
                # J1J2 J3J4
                [y, 0, y, y],
                dtype=np.float64,
            )
        )

        x += math.pi / update_rate
        time.sleep(update_period)


if __name__ == "__main__":
    main()
