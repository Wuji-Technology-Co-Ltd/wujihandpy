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

    # (also bulk-write) Enable all joints
    hand.write_joint_control_word(np.uint16(1))

    # Return all joints to initial point
    hand.write_joint_control_position(np.float64(0.0))

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
        y = (1 - math.cos(x)) * 0.8

        # Control index finger
        # Unchecked API is non-blocking (returns immediately, but success is not guaranteed)
        hand.finger(1).joint(0).write_joint_control_position_unchecked(np.float64(y))
        hand.finger(1).joint(2).write_joint_control_position_unchecked(np.float64(y))
        hand.finger(1).joint(3).write_joint_control_position_unchecked(np.float64(y))

        x += math.pi / update_rate
        time.sleep(update_period)


if __name__ == "__main__":
    main()
