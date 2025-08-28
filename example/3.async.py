import wujihandpy
import numpy as np
import math
import asyncio


async def main():
    hand = wujihandpy.Hand()
    try:
        await run(hand)
    finally:
        # Disable the entire hand
        await hand.write_joint_control_word_async(np.uint16(5))


async def run(hand: wujihandpy.Hand):
    # Set control mode
    await hand.write_joint_control_mode_async(np.uint16(2))

    # Enable whole hand
    await hand.write_joint_control_word_async(np.uint16(1))

    # Return all joints to initial point
    await hand.write_joint_control_position_async(
        np.array(
            [
                #   J1        J2        J3        J4
                [0x200000, 0x200000, 0x200000, 0x200000],  # F1
                [0xDFFFFF, 0xBFFFFF, 0x400000, 0x600000],  # F2
                [0xDFFFFF, 0x900000, 0x400000, 0x600000],  # F3
                [0xDFFFFF, 0x600000, 0x400000, 0x600000],  # F4
                [0xDFFFFF, 0x400000, 0x400000, 0x600000],  # F5
            ],
            dtype=np.int32,
        )
    )

    # Wait for joints to move into place
    await asyncio.sleep(0.5)

    # Disable unnecessary joints
    await hand.write_joint_control_word_async(
        np.array(
            [
                # J1 J2 J3 J4
                [5, 5, 5, 5],  # F1
                [1, 5, 1, 1],  # F2
                [1, 5, 1, 1],  # F3
                [5, 5, 5, 5],  # F4
                [5, 5, 5, 5],  # F5
            ],
            dtype=np.uint16,
        )
    )

    await asyncio.gather(shake(0.0, hand.finger(1)), shake(math.pi, hand.finger(2)))


async def shake(x: float, finger: wujihandpy.Finger):
    # 2Hz SDO Control
    update_rate = 2.0
    update_period = 1.0 / update_rate

    while True:
        x += math.pi / update_rate
        y = (math.cos(x) + 1.0) / 2.0

        pos_increase = np.int32(round(0xFFFFFF * y))
        pos_decrease = np.int32(0xFFFFFF - pos_increase)

        # Control finger
        finger.write_joint_control_position(
            np.array(
                #     J1      J2       J3            J4
                [pos_increase, 0, pos_decrease, pos_decrease],
                dtype=np.int32,
            )
        )

        await asyncio.sleep(update_period)


if __name__ == "__main__":
    asyncio.run(main())
