import numpy as np

BLOCKS_INIT_POSE = [0.08, -0.4, 0.0285, 0, 3.14159265, 0]
BAY_POSE = [0.266, 0.245, 0.0255, 0, 3.14159265, 0]
BAY_MID_POSE = [0.366, 0, 0.2255, 0, 3.14159265, 0]
BAY_OFFSET=[0.1166,0,0,0,0,0]

Z_DISTANCE = 0.05
Z_DISTANCE_OFFSET = np.array([0, 0, Z_DISTANCE, 0, 0, 0])
Y_BLOCK_WIDTH = np.array([0, 0.021, 0, 0, 0, 0])
BLOCK_Z_OFFSET = np.array([0, 0, 0.016, 0, 0, 0])

DELTA_BLOCK_OFFSET = np.array([0, -0.026, 0, 0, 0, 0])
DELTA_BLOCK_OFFSET2 = np.array([0, -0.026, 0.016, 0, 0, 0])


bay_bottom_pose = np.array(BAY_POSE)
bay_top_pose = bay_bottom_pose + 2 * Z_DISTANCE_OFFSET
bay_mid_pose = BAY_MID_POSE
bay_offset = np.array(BAY_OFFSET)


block_zero_bottom = np.array(BLOCKS_INIT_POSE)
block_zero_top = block_zero_bottom + Z_DISTANCE_OFFSET

#PISTON POSITIONS:
TOWER_TOP_WITHOUTSCALE = np.array ( [-1.018365208302633, -1.5090441156974812, -2.1208131313323975, -1.0830064874938508, 1.5720258951187134, -1.0213874022113245])
PISTON_INTIAL_POS2= np.array([-0.12598351709, -0.42600377839139036, 0.04407467212225197, 2.2213261533397977, 2.221121708971239, -0.00017191677339316886])
PISTON_INTIAL_POS= np.array([0.26401648291, -0.42600377839139036, 0.04407467212225197, 2.2213261533397977, 2.221121708971239, -0.00017191677339316886])