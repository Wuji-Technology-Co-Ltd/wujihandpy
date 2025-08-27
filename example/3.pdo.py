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
    # Set control mode & enable whole hand
    hand.write_joint_control_mode(np.uint16(2))
    hand.write_joint_control_word(np.uint16(1))

    # Calculate initial control position
    positions = hand.read_joint_position()
    fle_init = np.round(np.mean(positions[1:5, 0]))
    ext_init = 0xFFFFFF - fle_init

    # Return all joints to initial point
    hand.write_joint_control_position(
        np.array(
            [
                #   J1        J2        J3        J4
                [0x200000, 0x200000, 0x200000, 0x200000],  # F1
                [fle_init, 0xBFFFFF, ext_init, ext_init],  # F2
                [fle_init, 0x900000, ext_init, ext_init],  # F3
                [fle_init, 0x600000, ext_init, ext_init],  # F4
                [fle_init, 0x400000, ext_init, ext_init],  # F5
            ],
            dtype=np.int32,
        )
    )

    # Wait for joints to move into place
    time.sleep(0.1)

    # Disable the entire hand
    # Switching mode requires disabling then enabling
    hand.write_joint_control_word(np.uint16(5))

    # Switch to CSP mode & Enable PDO Control
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

    # 1kHz SDO Control
    update_rate = 1000.0
    update_period = 1.0 / update_rate

    x = math.acos((fle_init / 0xFFFFFF * 2) - 1)
    while True:
        x += math.pi / update_rate
        y = (math.cos(x) + 1.0) / 2.0

        flex_pos = np.int32(round(0xFFFFFF * y))
        extend_pos = np.int32(0xFFFFFF - flex_pos)

        hand.pdo_write_unchecked(
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

        time.sleep(update_period)


if __name__ == "__main__":
    main()
