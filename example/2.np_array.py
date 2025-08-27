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
    # Set control mode
    hand.write_joint_control_mode(np.uint16(2))

    # Enable whole hand
    hand.write_joint_control_word(np.uint16(1))

    # Return all joints to initial point
    hand.write_joint_control_position(
        np.array(
            [
                #   J1        J2        J3        J4
                [0x200000, 0x200000, 0x200000, 0x200000],  # F1
                [0xFFFFFF, 0xBFFFFF, 0x000000, 0x000000],  # F2
                [0xFFFFFF, 0x900000, 0x000000, 0x000000],  # F3
                [0xFFFFFF, 0x600000, 0x000000, 0x000000],  # F4
                [0xFFFFFF, 0x400000, 0x000000, 0x000000],  # F5
            ],
            dtype=np.int32,
        )
    )

    # Wait for joints to move into place
    time.sleep(0.5)

    # Disable the thumb
    hand.finger(0).write_joint_control_word(np.uint16(5))

    # Disable each J2
    for i in range(1, 5):
        hand.finger(i).joint(1).write_joint_control_word(np.uint16(5))

    # 1kHz SDO Control
    update_rate = 1000.0
    update_period = 1.0 / update_rate

    x = 0.0
    while True:
        x += 0.005
        y = (math.cos(x) + 1.0) / 2.0

        flex_pos = np.int32(round(0xFFFFFF * y))
        extend_pos = np.int32(0xFFFFFF - flex_pos)

        hand.write_joint_control_position_unchecked(
            np.array(
                [
                    [0, 0, 0, 0],
                    [flex_pos, 0, extend_pos, extend_pos],
                    [flex_pos, 0, extend_pos, extend_pos],
                    [flex_pos, 0, extend_pos, extend_pos],
                    [flex_pos, 0, extend_pos, extend_pos],
                ],
                dtype=np.int32,
            )
        )

        hand.trigger_transmission()

        time.sleep(update_period)


if __name__ == "__main__":
    main()
