import wujihandpy
import numpy as np
import asyncio

np.set_printoptions(formatter={"int": lambda x: hex(x)})


async def main():
    hand = wujihandpy.Hand()
    print(await hand.read_input_voltage_async())
    print(await hand.read_system_time_async())
    print(await hand.read_joint_position_async())

    while True:
        hand.read_system_time_async_unchecked()
        hand.trigger_transmission()
        print(hand.get_system_time())
        await asyncio.sleep(0.1)


asyncio.run(main())
