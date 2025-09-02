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
    # Set control mode (PP) & enable all joints
    hand.write_joint_control_mode(np.uint16(2))
    hand.write_joint_control_word(np.uint16(1))

    # Read joint limits and current positions
    joint_upper_limits = hand.read_joint_upper_limit()
    joint_lower_limits = hand.read_joint_lower_limit()
    current_positions = hand.read_joint_position()

    # Calculate normalized initial positions (0-1 range)
    normalized_positions = (current_positions - joint_lower_limits) / (
        joint_upper_limits - joint_lower_limits
    )

    # Calculate starting phase for cosine motion profile
    # (average of fingers 2-5, joints J1, J3, J4)
    initial_normalized = np.mean(normalized_positions[1:, (0, 2, 3)])
    x = math.acos((initial_normalized * 2) - 1)

    # Calculate target positions for homing motion
    target_positions = (
        initial_normalized * (joint_upper_limits - joint_lower_limits)
        + joint_lower_limits
    )
    # Reset thumb position
    target_positions[0] = 0
    # Set specific J2 positions for each finger
    target_positions[0:5, 1] = [0, 0.1, 0, 0, -0.1]

    # Execute homing motion
    hand.write_joint_control_position(target_positions)
    time.sleep(0.1)  # Wait for motion completion

    # Disable all joints
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
                [5, 5, 5, 5],  # F1
                [1, 5, 1, 1],  # F2
                [1, 5, 1, 1],  # F3
                [1, 5, 1, 1],  # F4
                [1, 5, 1, 1],  # F5
            ],
            dtype=np.uint16,
        )
    )

    # 1kHz PDO Control
    update_rate = 1000.0
    update_period = 1.0 / update_rate
    while True:
        y = (math.cos(x) + 1.0) / 2.0
        hand.pdo_write_unchecked(
            y * (joint_upper_limits - joint_lower_limits) + joint_lower_limits
        )

        x += math.pi / update_rate
        time.sleep(update_period)


if __name__ == "__main__":
    main()
