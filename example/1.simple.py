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
    # (bulk-write) Set control mode to PP (Point to Point Mode)
    # Equals to:
    # for i in range (0, 5):
    #     for j in range (0, 4):
    #         hand.finger(i).joint(j).write_joint_control_mode(np.uint16(2))
    # Normal APIs are blocking to ensure successful operations
    hand.write_joint_control_mode(np.uint16(2))

    # (also bulk-write) Enable whole hand
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
        hand.finger(i).joint(0).write_joint_control_position(np.int32(0xDFFFFF))
        hand.finger(i).joint(2).write_joint_control_position(np.int32(0x400000))
        hand.finger(i).joint(3).write_joint_control_position(np.int32(0x600000))

    # Wait for joints to move into place
    time.sleep(0.5)

    # Disable non-index fingers
    for i in range(0, 5):
        if i != 1:
            hand.finger(i).write_joint_control_word(np.uint16(5))

    # 2Hz SDO Control
    update_rate = 2.0
    update_period = 1.0 / update_rate

    x = 0.0
    while True:
        x += math.pi / update_rate
        y = (math.cos(x) + 1.0) / 2.0

        flex_pos = np.int32(round(0xFFFFFF * y))
        extend_pos = np.int32(0xFFFFFF - flex_pos)

        # Control index finger
        # Unchecked API is non-blocking (returns immediately, but success is not guaranteed)
        hand.finger(1).joint(0).write_joint_control_position_unchecked(flex_pos)
        hand.finger(1).joint(2).write_joint_control_position_unchecked(extend_pos)
        hand.finger(1).joint(3).write_joint_control_position_unchecked(extend_pos)

        # Unchecked API requires explicit triggering to save bandwidth
        hand.trigger_transmission()

        time.sleep(update_period)


if __name__ == "__main__":
    main()
