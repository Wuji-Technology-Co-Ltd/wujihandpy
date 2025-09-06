import wujihandpy
import numpy as np
import time
import math


def main():
    hand = wujihandpy.Hand(usb_pid=-1)
    try:
        run(hand)
    finally:
        # Disable the entire hand
        hand.write_joint_control_word(np.uint16(5))


def run(hand: wujihandpy.Hand):
    # Set control mode (PP) & enable all joints
    hand.write_joint_control_mode(np.uint16(2))
    hand.write_joint_control_word(np.uint16(1))

    # Return all joints to initial point
    hand.write_joint_control_position(
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

    # Disable the entire hand
    # Switching mode requires re-enabling
    hand.write_joint_control_word(np.uint16(5))

    # Configure Cyclic Synchronous Position (CSP) mode
    hand.write_joint_control_mode(np.uint16(4))  # Set joints to CSP mode
    hand.write_global_tpdo_id(np.uint16(1))  # Disable upstream PDO
    hand.write_pdo_interval(np.uint32(1000))  # 1ms update interval
    hand.write_pdo_enabled(np.uint8(1))  # Enable PDO communication
    hand.write_joint_control_word(np.uint16(1))  # Re-enable joints

    # Enable non-thumb fingers except J2
    hand.write_joint_control_word(
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

    # 1kHz PDO Control
    update_rate = 100.0
    update_period = 1.0 / update_rate

    x = 0
    while True:
        y = (1 - math.cos(x)) * 0.8

        # Unlike buffered SDO writes, PDO writes are:
        # - Non-blocking
        # - Sent immediately
        # - Not delivery-guaranteed
        # Therefore, continuous sending is required
        hand.pdo_write_unchecked(
            np.array(
                [
                    [0.975, 0.523, 0.271, -0.45],
                    [0.382, 0.241, -0.003, -0.275],
                    [-0.299, 0.329, 0.067, -0.286],
                    [-0.122, 0.228, 0.315, -0.178],
                    [0.205, 0.087, 0.288, -0.149],
                ]
            )
        )
        # Equals to:
        # hand.pdo_write_unchecked(np.float64(y))

        x += math.pi / update_rate
        time.sleep(update_period)


if __name__ == "__main__":
    main()
