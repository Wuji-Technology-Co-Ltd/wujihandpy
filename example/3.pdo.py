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
        hand.write_joint_control_word(np.uint16(5))


def run(hand: wujihandpy.Hand):
    # Set control mode (PP) & enable whole hand
    hand.write_joint_control_mode(np.uint16(2))
    hand.write_joint_control_word(np.uint16(1))

    # Calculate initial control position
    positions = hand.read_joint_position()
    init_inc = np.round(np.mean(positions[1:5, 0]))
    init_dec = 0xFFFFFF - init_inc

    # Return all joints to initial point
    hand.write_joint_control_position(
        np.array(
            [
                #   J1        J2        J3        J4
                [0x200000, 0x200000, 0x200000, 0x200000],  # F1
                [init_inc, 0xBFFFFF, init_dec, init_dec],  # F2
                [init_inc, 0x900000, init_dec, init_dec],  # F3
                [init_inc, 0x600000, init_dec, init_dec],  # F4
                [init_inc, 0x400000, init_dec, init_dec],  # F5
            ],
            dtype=np.int32,
        )
    )

    # Wait for joints to move into place
    time.sleep(0.1)

    # Disable the entire hand
    # Switching mode requires re-enabling
    hand.write_joint_control_word(np.uint16(5))

    # Switch to CSP (Cyclic Synchronous Position) mode & Enable PDO Control
    hand.write_joint_control_mode(np.uint16(4))
    hand.write_global_tpdo_id(np.uint16(1))
    hand.write_pdo_interval(np.uint32(950))
    hand.write_pdo_enabled(np.uint8(1))
    hand.write_joint_control_word(np.uint16(1))

    # Disable the thumb
    hand.finger(0).write_joint_control_word(np.uint16(5))

    # Disable each J2
    for i in range(1, 5):
        hand.finger(i).joint(1).write_joint_control_word(np.uint16(5))

    # 1kHz PDO Control
    update_rate = 1000.0
    update_period = 1.0 / update_rate

    x = math.acos((init_inc / 0xFFFFFF * 2) - 1)
    while True:
        x += math.pi / update_rate
        y = (math.cos(x) + 1.0) / 2.0

        pos_increase = np.int32(round(0xFFFFFF * y))
        pos_decrease = np.int32(0xFFFFFF - pos_increase)

        hand.pdo_write_unchecked(
            np.array(
                [
                    [0, 0, 0, 0],
                    [pos_increase, 0, pos_decrease, pos_decrease],
                    [pos_increase, 0, pos_decrease, pos_decrease],
                    [pos_increase, 0, pos_decrease, pos_decrease],
                    [pos_increase, 0, pos_decrease, pos_decrease],
                ],
                dtype=np.int32,
            )
        )

        time.sleep(update_period)


if __name__ == "__main__":
    main()
