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
    hand.finger(0).joint(0).write_joint_control_position(np.int32(0x200000))
    hand.finger(0).joint(1).write_joint_control_position(np.int32(0x200000))
    hand.finger(0).joint(2).write_joint_control_position(np.int32(0x200000))
    hand.finger(0).joint(3).write_joint_control_position(np.int32(0x200000))
    hand.finger(1).joint(1).write_joint_control_position(np.int32(0xBFFFFF))
    hand.finger(2).joint(1).write_joint_control_position(np.int32(0x900000))
    hand.finger(3).joint(1).write_joint_control_position(np.int32(0x600000))
    hand.finger(4).joint(1).write_joint_control_position(np.int32(0x400000))
    for i in range(1, 5):
        hand.finger(i).joint(0).write_joint_control_position(np.int32(0xFFFFFF))
        hand.finger(i).joint(2).write_joint_control_position(np.int32(0x000000))
        hand.finger(i).joint(3).write_joint_control_position(np.int32(0x000000))

    # Wait for joints to move into place
    time.sleep(0.5)

    # Disable the thumb
    hand.finger(0).write_joint_control_word(np.uint16(5))

    # 1kHz SDO Control
    update_rate = 1000.0
    update_period = 1.0 / update_rate

    x = 0.0
    while True:
        x += 0.005
        y = (math.cos(x) + 1.0) / 2.0
        pos = np.int32(round(0xFFFFFF * y))
        pos_reverse = np.int32(0xFFFFFF - pos)
        for i in range(1, 5):
            hand.finger(i).joint(0).write_joint_control_position_unchecked(pos)
            hand.finger(i).joint(2).write_joint_control_position_unchecked(pos_reverse)
            hand.finger(i).joint(3).write_joint_control_position_unchecked(pos_reverse)
        hand.trigger_transmission()

        time.sleep(update_period)


if __name__ == "__main__":
    main()
