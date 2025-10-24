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
        await hand.write_joint_enabled_async(False)


async def run(hand: wujihandpy.Hand):
    # Enable all joints
    await hand.write_joint_enabled_async(True)

    # Return all joints to initial point
    await hand.write_joint_target_position_async(
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
    await asyncio.sleep(0.5)

    # Disable unnecessary joints
    await hand.write_joint_enabled_async(
        np.array(
            [
                # J1J2 J3J4
                [0, 0, 0, 0],  # F1
                [1, 0, 1, 1],  # F2
                [1, 0, 1, 1],  # F3
                [0, 0, 0, 0],  # F4
                [0, 0, 0, 0],  # F5
            ],
            dtype=bool,
        )
    )

    await asyncio.gather(shake(0.0, hand.finger(1)), shake(math.pi, hand.finger(2)))


async def shake(x: float, finger: wujihandpy.Finger):
    # 2Hz SDO Control
    update_rate = 2.0
    update_period = 1.0 / update_rate

    while True:
        y = (1 - math.cos(x)) * 0.8

        # Control middle finger
        await finger.write_joint_target_position_async(
            np.array(
                # J1J2 J3J4
                [y, 0, y, y],
                dtype=np.float64,
            )
        )

        x += math.pi / update_rate
        await asyncio.sleep(update_period)


if __name__ == "__main__":
    asyncio.run(main())
