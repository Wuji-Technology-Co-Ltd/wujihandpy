import wujihandpy

hand = wujihandpy.Hand()

# Read
print("Input voltage: ", hand.read_input_voltage())

# Bulk-Read: Return 5x4 np.array
print("Motor temperatures: \n", hand.read_joint_temperature())

# Normal APIs are blocking to ensure successful operations